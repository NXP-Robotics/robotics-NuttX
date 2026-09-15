/****************************************************************************
 *
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* NXP NETC VSI-to-PSI mailbox messaging.
 *
 * A VSI cannot write the PSI's shared MAC/filter/link registers, so it
 * requests those operations from the PSI owner over the VSI-to-PSI message
 * channel.  This file owns the mailbox transport and per-operation encoders;
 * imx9_netc_vsi.c keeps the interface/lifecycle policy.
 *
 * Ported from Zephyr eth_nxp_imx_netc_msg.c (Apache-2.0).
 * OS changes:
 *   k_mutex_lock/unlock  -> nxmutex_lock/unlock
 *   k_msleep             -> nxsig_usleep  (in thread context)
 *   sys_timepoint_*      -> clock_gettime / CLOCK_MONOTONIC arithmetic
 *   crc16_itu_t(0xFFFF)  -> crc16part(src, len, 0xFFFF) (nuttx/crc16.h)
 *
 * msg_buff placement:
 *   msg_buff is placed in DDR via the .netc_dma linker section (NOLOAD,
 *   remoteproc-loadable).  DDR is covered by the M7 D-cache, so the buffer
 *   must be clean-flushed to DDR before submitting to NETC DMA, so that the
 *   PSI DMA engine reads the written content and not stale cache lines.
 *   The physical DDR address is passed directly to EP_VsiSendMsg without
 *   any address translation (FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET is not
 *   defined for iMX95 M7).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/mutex.h>
#include <nuttx/crc16.h>

#include "imx9_netc_priv.h"
#include "imx9_netc_msg.h"

/* NXP MCUX SDK NETC HAL message definitions */

#include "fsl_netc_msg.h"

#ifdef CONFIG_ARMV7M_DCACHE
#  include <nuttx/cache.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETC_MSG_RESP_CLASS(r)  ((uint16_t)(r) >> 8)
#define NETC_MSG_RESP_CODE(r)   (((uint16_t)(r) >> 4) & 0x0fu)

/* Alignment of each VSI->PSI message within the 64-byte buffer */

#define NETC_MSG_VSI_ALIGN  32u

/* Timeout waiting for PSI to consume a VSI->PSI message (ms) */

#define NETC_MSG_VSI_TIMEOUT_MS  100

/* MAC/VLAN filter message fields */

#define NETC_MAC_HASH_TABLE_SIZE_64   0u
#define NETC_VLAN_HASH_TABLE_SIZE_64  0u

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_netc_vsi_msg_send
 *
 * Description:
 *   Send a VSI->PSI control message and poll for the PSI reply.
 *   Acquires priv->msg_lock (recursive) for the duration of the send.
 *   The caller may already hold msg_lock; re-entrant acquisition is safe.
 *
 * Returned Value:
 *   0 on success, -EBADMSG if the PSI returned an unexpected class,
 *   -ETIMEDOUT if the PSI did not respond within NETC_MSG_VSI_TIMEOUT_MS,
 *   -EBUSY / -EIO on other send errors.
 *
 ****************************************************************************/

static int imx9_netc_vsi_msg_send(struct imx9_netc_vsi_s *priv,
                                  uint8_t class_id, uint8_t cmd_id,
                                  const void *payload, size_t payload_len,
                                  uint8_t want_class, uint16_t *resp)
{
  ep_handle_t            *handle  = &priv->handle;
  netc_msg_header_t      *hdr     = (netc_msg_header_t *)priv->msg_buff;
  size_t                  msg_len;
  netc_vsi_msg_tx_status_t status;
  struct timespec         deadline;
  struct timespec         now;
  uint16_t                crc;
  int                     ret;

  msg_len = (sizeof(*hdr) + payload_len + NETC_MSG_VSI_ALIGN - 1u) &
            ~(NETC_MSG_VSI_ALIGN - 1u);

  if (msg_len > NETC_MSG_VSI_BUF_SIZE)
    {
      return -EINVAL;
    }

  nxrmutex_lock(&priv->msg_lock);

  memset(priv->msg_buff, 0, msg_len);
  hdr->classId = class_id;
  hdr->cmdId   = cmd_id;
  hdr->len     = (uint8_t)(msg_len / NETC_MSG_VSI_ALIGN - 1u);

  if (payload_len != 0u && payload != NULL)
    {
      memcpy(priv->msg_buff + sizeof(*hdr), payload, payload_len);
    }

  /* CRC16 covers bytes [2..msg_len-1]; bytes 0-1 carry the CRC.
   * Use poly 0x1021, initial seed 0xFFFF to match crc16_itu_t(0xFFFF, ...)
   * used by the Zephyr PSI driver.  NuttX crc16() uses seed 0x0000 which
   * produces a different checksum and causes the PSI to reject every message. */

  crc = crc16part((const uint8_t *)(priv->msg_buff + 2), msg_len - 2u,
                  0xFFFFu);
  priv->msg_buff[0] = (uint8_t)(crc >> 8);
  priv->msg_buff[1] = (uint8_t)(crc & 0xffu);

  /* Flush msg_buff lines from D-cache to DDR before NETC DMA reads them.
   * msg_buff is in cacheable DDR; without this flush the PSI DMA engine
   * may read stale data from DDR.
   */

#ifdef CONFIG_ARMV7M_DCACHE
  up_clean_dcache((uintptr_t)priv->msg_buff,
                  (uintptr_t)priv->msg_buff + msg_len);
#endif

  if (EP_VsiSendMsg(handle,
                    (uint64_t)(uintptr_t)priv->msg_buff,
                    msg_len) != kStatus_Success)
    {
      ret = -EBUSY;
      goto out;
    }

  /* Compute deadline = now + NETC_MSG_VSI_TIMEOUT_MS */

  clock_gettime(CLOCK_MONOTONIC, &deadline);
  deadline.tv_nsec += (long)(NETC_MSG_VSI_TIMEOUT_MS * 1000000L);
  if (deadline.tv_nsec >= 1000000000L)
    {
      deadline.tv_sec++;
      deadline.tv_nsec -= 1000000000L;
    }

  for (;;)
    {
      NETC_SIVsiCheckTxStatus(handle->hw.si, &status);
      if (!status.txBusy)
        {
          break;
        }

      clock_gettime(CLOCK_MONOTONIC, &now);
      if (now.tv_sec > deadline.tv_sec ||
          (now.tv_sec == deadline.tv_sec &&
           now.tv_nsec >= deadline.tv_nsec))
        {
          nwarn("VSI->PSI: class=0x%02x timed out\n", class_id);
          ret = -ETIMEDOUT;
          goto out;
        }

      nxsig_usleep(1000);  /* 1 ms */
    }

  if (status.isTxErr)
    {
      nwarn("VSI->PSI: class=0x%02x send error code=0x%04x\n",
            class_id, status.msgCode);
      ret = -EIO;
      goto out;
    }

  *resp = status.msgCode;
  ret = (NETC_MSG_RESP_CLASS(*resp) == want_class) ? 0 : -EBADMSG;

out:
  nxrmutex_unlock(&priv->msg_lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_netc_vsi_msg_set_primary_mac
 ****************************************************************************/

int imx9_netc_vsi_msg_set_primary_mac(struct imx9_netc_vsi_s *priv)
{
  uint16_t resp;
  int      ret;

  struct
    {
      uint8_t count;
      uint8_t reserved[3];
      uint8_t mac[6];
    } req;

  memset(&req, 0, sizeof(req));
  memcpy(req.mac, priv->mac_addr, sizeof(req.mac));

  ret = imx9_netc_vsi_msg_send(priv,
                                (uint8_t)kNETC_MsgClassMacFilter,
                                (uint8_t)kNETC_MsgMacFilterSetMacAddr,
                                &req, sizeof(req),
                                (uint8_t)kNETC_MsgClassDone, &resp);
  if (ret == -EBADMSG)
    {
      ninfo("VSI MAC-filter request rejected by PSI: 0x%04x\n", resp);
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: imx9_netc_vsi_msg_set_mac_hash
 ****************************************************************************/

int imx9_netc_vsi_msg_set_mac_hash(struct imx9_netc_vsi_s *priv)
{
  uint16_t resp;
  int      ret;

  struct
    {
      uint8_t  size_type;
      uint8_t  reserved[7];
      uint64_t hash_tbl;
    } req;

  memset(&req, 0, sizeof(req));
  req.size_type = (uint8_t)((NETC_MAC_HASH_TABLE_SIZE_64 & 0x3fu) |
                             (uint8_t)(NETC_MAC_FILTER_TYPE_MC << 6));
  req.hash_tbl  = priv->mc_hash;

  ret = imx9_netc_vsi_msg_send(priv,
                                (uint8_t)kNETC_MsgClassMacFilter,
                                (uint8_t)kNETC_MsgMacFilterSet,
                                &req, sizeof(req),
                                (uint8_t)kNETC_MsgClassDone, &resp);
  if (ret == -EBADMSG)
    {
      ninfo("VSI MC-hash rejected by PSI: 0x%04x\n", resp);
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: imx9_netc_vsi_msg_set_vlan_hash
 ****************************************************************************/

int imx9_netc_vsi_msg_set_vlan_hash(struct imx9_netc_vsi_s *priv)
{
  uint16_t resp;
  int      ret;

  struct
    {
      uint8_t  size;
      uint8_t  reserved[7];
      uint64_t hash_tbl;
    } req;

  memset(&req, 0, sizeof(req));
  req.size     = NETC_VLAN_HASH_TABLE_SIZE_64;
  req.hash_tbl = priv->vlan_hash;

  ret = imx9_netc_vsi_msg_send(priv,
                                (uint8_t)kNETC_MsgClassVlanFilter,
                                (uint8_t)kNETC_MsgVlanFilterSet,
                                &req, sizeof(req),
                                (uint8_t)kNETC_MsgClassDone, &resp);
  if (ret == -EBADMSG)
    {
      ninfo("VSI VLAN-hash rejected by PSI: 0x%04x\n", resp);
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: imx9_netc_vsi_msg_set_mac_promisc
 ****************************************************************************/

int imx9_netc_vsi_msg_set_mac_promisc(struct imx9_netc_vsi_s *priv,
                                      uint8_t type_mask, bool enable)
{
  uint16_t resp;
  int      ret;

  struct
    {
      uint8_t flush_macs  : 1;
      uint8_t promisc_mode: 1;
      uint8_t reserved    : 4;
      uint8_t type        : 2;
    } req;

  memset(&req, 0, sizeof(req));
  req.promisc_mode = enable ? 1u : 0u;
  req.type         = type_mask;

  ret = imx9_netc_vsi_msg_send(priv,
                                (uint8_t)kNETC_MsgClassMacFilter,
                                (uint8_t)kNETC_MsgMacFilterSetMacPromisc,
                                &req, sizeof(req),
                                (uint8_t)kNETC_MsgClassDone, &resp);
  if (ret == -EBADMSG)
    {
      ninfo("VSI promisc request rejected by PSI: 0x%04x\n", resp);
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: imx9_netc_vsi_msg_enable_link_notify
 ****************************************************************************/

int imx9_netc_vsi_msg_enable_link_notify(struct imx9_netc_vsi_s *priv)
{
  uint16_t resp;
  int      ret;

  ret = imx9_netc_vsi_msg_send(priv,
                                (uint8_t)kNETC_MsgClassLinkStatus,
                                (uint8_t)kNETC_MsgLinkStatusEnableNotify,
                                NULL, 0,
                                (uint8_t)kNETC_MsgClassDone, &resp);
  if (ret == -EBADMSG)
    {
      nwarn("VSI link-notify rejected by PSI: 0x%04x\n", resp);
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: imx9_netc_vsi_msg_poll_link
 ****************************************************************************/

enum imx9_netc_msg_link_e
imx9_netc_vsi_msg_poll_link(struct imx9_netc_vsi_s *priv)
{
  ep_handle_t               *handle = &priv->handle;
  enum imx9_netc_msg_link_e  link   = IMX9_NETC_MSG_LINK_NONE;
  uint16_t                   msg    = 0;

  if ((EP_VsiGetStatus(handle) & (uint32_t)kNETC_VsiMsgRxFlag) == 0u)
    {
      return IMX9_NETC_MSG_LINK_NONE;
    }

  if (EP_VsiReceiveMsg(handle, &msg) == kStatus_Success &&
      NETC_MSG_RESP_CLASS(msg) == (uint16_t)kNETC_MsgClassLinkStatus)
    {
      link = (NETC_MSG_RESP_CODE(msg) == (uint16_t)kNETC_MsgLinkStatusUp)
               ? IMX9_NETC_MSG_LINK_UP
               : IMX9_NETC_MSG_LINK_DOWN;
    }

  EP_VsiClearStatus(handle, (uint32_t)kNETC_VsiMsgRxFlag);

  return link;
}

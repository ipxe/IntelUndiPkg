/**************************************************************************

Copyright (c) 2016 - 2025, Intel Corporation. All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/
/** @file
  Header file containing definitions of branding strings
  supported by the driver.
**/

#include "DeviceSupport.h"

BRAND_STRUCT mBrandingTable[] = {
#ifndef NO_BRANDING_SUPPORT
  /* {Vendor ID, SubVendor ID, Device ID, SubSystem ID, Branding String} */
#ifdef SIMICS_SUPPORT
  {0x8086, 0x0000, 0x18E4, 0x0000, L"Intel(R) Ethernet Controller ICE SIMICS NIC ID"},
  {0x8086, 0x0000, 0x18ED, 0x0000, L"Intel(R) Ethernet Controller ICE SIMICS SWITCH ID"},
#endif /* SIMICS_SUPPORT */
#ifdef E810C_SUPPORT
  /* Intel(R) Ethernet Controller E810-C for backplane */
  {0x8086, 0x0000, 0x1591, 0x0000, L"Intel(R) Ethernet Controller E810-C for backplane"},
  /* Intel(R) Ethernet Controller E810-C for QSFP */
  {0x8086, 0x0000, 0x1592, 0x0000, L"Intel(R) Ethernet Controller E810-C for QSFP"},
  /* Intel(R) Ethernet Network Adapter E810-C-Q1 */
  {0x8086, 0x8086, 0x1592, 0x0001, L"Intel(R) Ethernet Network Adapter E810-C-Q1"},
  /* Intel(R) Ethernet Network Adapter E810-C-Q2 */
  {0x8086, 0x8086, 0x1592, 0x0002, L"Intel(R) Ethernet Network Adapter E810-C-Q2"},
  /* Intel(R) Ethernet Network Adapter E810-C-Q1 for OCP 3.0 */
  {0x8086, 0x8086, 0x1592, 0x0005, L"Intel(R) Ethernet Network Adapter E810-C-Q1 for OCP 3.0"},
  /* Intel(R) Ethernet Network Adapter E810-C-Q2 for OCP 3.0 */
  {0x8086, 0x8086, 0x1592, 0x0006, L"Intel(R) Ethernet Network Adapter E810-C-Q2 for OCP 3.0"},
  /* Bowman Flat Single Port QSFP - OEM Gen (Columbiaville)  */
  {0x8086, 0x8086, 0x1592, 0x000A, L"Intel(R) Ethernet Network Adapter E810-C-Q1 for OCP"},
  /* Island Rapids DP, 100G QSFP28, bifurcated 2x CVL - OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1592, 0x000E, L"Intel(R) Ethernet Network Adapter E810-2C-Q2"},
  /* Logan Beach 8x10 QSFP with PtP & SyncE - OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1592, 0x000F, L"Intel(R) Ethernet Network Adapter E810-C-Q2T"},
  /* Empire Flat (E810-L) - 1x50G QSFP OCP 3.0, SP, OEM Gen (Columbiaville)  */
  {0x8086, 0x8086, 0x1592, 0x0011, L"Intel(R) Ethernet Network Adapter E810-C-Q1 for OCP 3.0"},
  /* Empire Flat (E810-C) - QSFP OCP 3.0, SP, Apple (Columbiaville) */
  {0x8086, 0x8086, 0x1592, 0x0013, L"Intel(R) Ethernet Network Adapter E810-C-Q1 for OCP 3.0"},
  /* Intel(R) Ethernet Controller E810-C for SFP */
  {0x8086, 0x0000, 0x1593, 0x0000, L"Intel(R) Ethernet Controller E810-C for SFP"},
  /* Mentor Harbor Dual Port SFP - OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1593, 0x0002, L"Intel(R) Ethernet Network Adapter E810-L-2"},
  /* Intel(R) Ethernet Network Adapter E810-XXV-4 */
  {0x8086, 0x8086, 0x1593, 0x0005, L"Intel(R) Ethernet Network Adapter E810-XXV-4"},
  /* Intel(R) Ethernet Network Adapter E810-XXV-4 */
  {0x8086, 0x8086, 0x1593, 0x0007, L"Intel(R) Ethernet Network Adapter E810-XXV-4"},
  /* Silver Flat (E810-L) - SFP28 OCP 3.0, DP, OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1593, 0x000B, L"Intel(R) Ethernet Network Adapter E810-L-2 for OCP 3.0"},
  /* Meadow Flat (E810-XXV-4) - SFP28 OCP 3.0, QP, OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1593, 0x000C, L"Intel(R) Ethernet Network Adapter E810-XXV-4 for OCP 3.0"},
  /* Westport Channel Quad Port SFP with PtP & SyncE - OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1593, 0x000E, L"Intel(R) Ethernet Network Adapter E810-XXV-4T"},
  /* Cisco Tacoma Rapids Dual Port QSFP - OEM Gen (Columbiaville) */
  {0x8086, 0x1137, 0x1592, 0x02BF, L"Cisco(R) E810CQDA2 2x100 GbE QSFP28 PCIe NIC"},
  /* Cisco Salem Channel Quad Port SFP - OEM Gen (Columbiaville) */
  {0x8086, 0x1137, 0x1593, 0x02C3, L"Cisco(R) E810XXVDA4 4x25/10 GbE SFP28 PCIe NIC"},
  /* Cisco Westport Channel (w/GPS) Quad Port SFP with PtP & SyncE */
  {0x8086, 0x1137, 0x1593, 0x02E9, L"Cisco(R) E810XXVDA4TG 4x25/10 GbE SFP28 PCIe NIC"},
  /* Cisco Westport Channel (w/o GPS) Quad Port SFP with PtP & SyncE */
  {0x8086, 0x1137, 0x1593, 0x02EA, L"Cisco(R) E810XXVDA4T 4x25/10 GbE SFP28 PCIe NIC"},
#endif /* E810C_SUPPORT */
#ifdef E810_XXV_SUPPORT
  /* Intel(R) Ethernet Controller E810-XXV for backplane */
  {0x8086, 0x0000, 0x1599, 0x0000, L"Intel(R) Ethernet Controller E810-XXV for backplane"},
  /* Intel(R) Ethernet Controller E810-XXV for QSFP */
  {0x8086, 0x0000, 0x159A, 0x0000, L"Intel(R) Ethernet Controller E810-XXV for QSFP"},
  /* Intel(R) Ethernet Controller E810-XXV for SFP */
  {0x8086, 0x0000, 0x159B, 0x0000, L"Intel(R) Ethernet Controller E810-XXV for SFP"},
  /* Intel(R) Ethernet Network Adapter E810-XXV-2 */
  {0x8086, 0x8086, 0x159B, 0x0003, L"Intel(R) Ethernet Network Adapter E810-XXV-2"},
  /* Intel(R) Ethernet Network Adapter E810-XXV-2 for OCP 3.0 */
  {0x8086, 0x8086, 0x159B, 0x0005, L"Intel(R) Ethernet Network Adapter E810-XXV-2 for OCP 3.0"},
  /* Cisco Clifton Channel dual port SFP OEM Gen (Columbiaville SD) */
  {0x8086, 0x1137, 0x159B, 0x02BE, L"Cisco(R) E810XXVDA2 2x25/10 GbE SFP28 PCIe NIC"},
  /* Albany Channel Dual Port SFP - OEM Gen (Columbiaville) */
  {0x8086, 0x8086, 0x1593, 0x0008, L"Intel(R) Ethernet Network Adapter E810-XXV-2"},
  /* OEMGen Salem Channel Quad Port SFP w Reversed Port Order */
  {0x8086, 0x8086, 0x1593, 0x0012, L"Intel(R) Ethernet 25G 4P E810-XXV Adapter"},
#endif /* E810_XXV_SUPPORT */
#ifdef E830_SUPPORT
  /* Connorsville (E830) - Backplane */
  {0x8086, 0x0000, 0x12D1, 0x0000, L"Intel(R) Ethernet Controller E830-CC for backplane"},
  /* Connorsville (E830) - Backplane - 100G SKU */
  {0x8086, 0x0000, 0x12D5, 0x0000, L"Intel(R) Ethernet Controller E830-C for backplane"},
  /* Connorsville (E830) - Backplane - 50G SKU */
  {0x8086, 0x0000, 0x12DC, 0x0000, L"Intel(R) Ethernet Controller E830-L for backplane"},
  /* Connorsville (E830) - QSFP56 */
  {0x8086, 0x0000, 0x12D2, 0x0000, L"Intel(R) Ethernet Controller E830-CC for QSFP"},
  /* Connorsville (E830) - Jasper Beach DP (200G QSFP) */
  {0x8086, 0x8086, 0x12D2, 0x0001, L"Intel(R) Ethernet Network Adapter E830-C-Q2"},
  /* Connorsville (E830) - Hunters Flat DP (200G QSFP) */
  {0x8086, 0x8086, 0x12D2, 0x0002, L"Intel(R) Ethernet Network Adapter E830-C-Q2 for OCP 3.0"},
  /* Connorsville (E830) - Jasper Beach SP (200G QSFP) */
  {0x8086, 0x8086, 0x12D2, 0x0003, L"Intel(R) Ethernet Network Adapter E830-CC-Q1"},
  /* Connorsville (E830) - Hunters Flat SP (200G QSFP) */
  {0x8086, 0x8086, 0x12D2, 0x0004, L"Intel(R) Ethernet Network Adapter E830-CC-Q1 for OCP 3.0"},
  /* Connorsville (E830) - QSFP56 - 100G SKU */
  {0x8086, 0x0000, 0x12D8, 0x0000, L"Intel(R) Ethernet Controller E830-C for QSFP"},
  /* Connorsville (E830) - QSFP56 - 50G SKU */
  {0x8086, 0x0000, 0x12DD, 0x0000, L"Intel(R) Ethernet Controller E830-L for QSFP"},
  /* Connorsville (E830) - SFP28 */
  {0x8086, 0x0000, 0x12D3, 0x0000, L"Intel(R) Ethernet Controller E830-CC for SFP"},
  /* Connorsville (E830) - McKenzie Flat (25G SFP) - Pre-prod */
  {0x8086, 0x8086, 0x12D3, 0x0001, L"Intel(R) Ethernet Network Adapter E830-XXV-2 for OCP 3.0"},
  /* Connorsville (E830) - Arcata Channel (25G SFP) - Preprod */
  {0x8086, 0x8086, 0x12D3, 0x0003, L"Intel(R) Ethernet Network Adapter E830-XXV-2"},
  /* Connorsville (E830) - Bailey Flat (25G SFP) */
  {0x8086, 0x8086, 0x12D3, 0x0004, L"Intel(R) Ethernet Network Adapter E830-XXV-4 for OCP 3.0"},
  /* Connorsville (E830) - Carter Flat */
  {0x8086, 0x8086, 0x12D3, 0x0005, L"Intel(R) Ethernet Network Adapter E830-XXV-8F for OCP 3.0"},
  /* Connorsville (E830) - Pine Channel */
  {0x8086, 0x8086, 0x12D3, 0x0007, L"Intel(R) Ethernet Network Adapter E830-XXV-4F"},
  /* Connorsville (E830) - SFP28 - 100G SKU */
  {0x8086, 0x0000, 0x12DA, 0x0000, L"Intel(R) Ethernet Controller E830-C for SFP"},
  /* Connorsville (E830) - McKenzie Flat (25G SFP) */
  {0x8086, 0x8086, 0x12DE, 0x0001, L"Intel(R) Ethernet Network Adapter E830-XXV-2 for OCP 3.0"},
  /* Connorsville (E830) - Arcata Channel (25G SFP) */
  {0x8086, 0x8086, 0x12DE, 0x0003, L"Intel(R) Ethernet Network Adapter E830-XXV-2"},
  /* Connorsville (E830) - SFP28 - 50G SKU */
  {0x8086, 0x0000, 0x12DE, 0x0000, L"Intel(R) Ethernet Controller E830-L for SFP"},
  /* Connorsville (E830) - SFP-DD */
  {0x8086, 0x0000, 0x12D4, 0x0000, L"Intel(R) Ethernet Controller E830-CC for SFP-DD"},
#endif /* E830_SUPPORT */
#ifdef E835_SUPPORT
  /* DCR 4973 - Connorsville (E835) - Backplane */
  {0x8086, 0x0000, 0x1248, 0x0000, L"Intel(R) Ethernet Controller E835-CC for backplane"},
  /* DCR 4973 - Connorsville (E835) - Backplane - 100G SKU */
  {0x8086, 0x0000, 0x1261, 0x0000, L"Intel(R) Ethernet Controller E835-C for backplane"},
  /* DCR 4973 - Connorsville (E835) - Backplane - 50G SKU */
  {0x8086, 0x0000, 0x1265, 0x0000, L"Intel(R) Ethernet Controller E835-L for backplane"},
  /* DCR 4973 - Connorsville (E835) - QSFP56 */
  {0x8086, 0x0000, 0x1249, 0x0000, L"Intel(R) Ethernet Controller E835-CC for QSFP"},
  /* DCR 4973 - Connorsville (E835) - QSFP56 - 100G SKU */
  {0x8086, 0x0000, 0x1262, 0x0000, L"Intel(R) Ethernet Controller E835-C for QSFP"},
  /* DCR 4973 - Connorsville (E835) - QSFP56 - 50G SKU */
  {0x8086, 0x0000, 0x1266, 0x0000, L"Intel(R) Ethernet Controller E835-L for QSFP"},
  /* DCR 4973 - Connorsville (E835) - SFP28 */
  {0x8086, 0x0000, 0x124A, 0x0000, L"Intel(R) Ethernet Controller E835-CC for SFP"},
  /* DCR 4973 - Connorsville (E835) - SFP28 - 100G SKU */
  {0x8086, 0x0000, 0x1263, 0x0000, L"Intel(R) Ethernet Controller E835-C for SFP"},
  /* DCR 4973 - Connorsville (E835) - SFP28 - 50G SKU */
  {0x8086, 0x0000, 0x1267, 0x0000, L"Intel(R) Ethernet Controller E835-L for SFP"},
  /* DCR 5002 - Connorsville (E835) - Jasper Beach DP (200G QSFP) */
  {0x8086, 0x8086, 0x1249, 0x0001, L"Intel(R) Ethernet Network Adapter E835-C-Q2"},
  /* DCR 5000 - Connorsville (E835) - Hunters Flat DP (200G QSFP) */
  {0x8086, 0x8086, 0x1249, 0x0002, L"Intel(R) Ethernet Network Adapter E835-C-Q2 for OCP 3.0"},
  /* DCR 5011 - Connorsville (E835) - Jasper Beach SP (200G QSFP) */
  {0x8086, 0x8086, 0x1249, 0x0003, L"Intel(R) Ethernet Network Adapter E835-CC-Q1"},
  /* DCR 5010 - Connorsville (E835) - Hunters Flat SP (200G QSFP) */
  {0x8086, 0x8086, 0x1249, 0x0004, L"Intel(R) Ethernet Network Adapter E835-CC-Q1 for OCP 3.0"},
  /* DCR 5003 - Connorsville (E835) - McKenzie Flat (25G SFP) */
  {0x8086, 0x8086, 0x124A, 0x0001, L"Intel(R) Ethernet Network Adapter E835-XXV-2 for OCP 3.0"},
  /* DCR 4986 - Connorsville (E835) - Stanley Channel (25G SFP) */
  {0x8086, 0x8086, 0x124A, 0x0002, L"Intel(R) Ethernet Network Adapter E835-XXV-4"},
  /* DCR 4999 - Connorsville (E835) - Arcata Channel (25G SFP) */
  {0x8086, 0x8086, 0x124A, 0x0003, L"Intel(R) Ethernet Network Adapter E835-XXV-2"},
  /* DCR 5001 - Connorsville (E835) - Bailey Flat (25G SFP) */
  {0x8086, 0x8086, 0x124A, 0x0004, L"Intel(R) Ethernet Network Adapter E835-XXV-4 for OCP 3.0"},
#endif /* E835_SUPPORT */
#else /* NOT N0_BRANDING_SUPPORT */
  {0x8086, 0x8086, 0x0000, 0x0000, L"Intel(R) Network Connection"},
#endif /* N0_BRANDING_SUPPORT */
  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, L" "},

};

UINTN mBrandingTableSize = (sizeof (mBrandingTable) / sizeof (mBrandingTable[0]));


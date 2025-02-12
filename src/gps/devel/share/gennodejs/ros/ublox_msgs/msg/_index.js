
"use strict";

let NavCLOCK = require('./NavCLOCK.js');
let RxmSFRB = require('./RxmSFRB.js');
let CfgSBAS = require('./CfgSBAS.js');
let EsfSTATUS = require('./EsfSTATUS.js');
let NavDGPS_SV = require('./NavDGPS_SV.js');
let NavPOSLLH = require('./NavPOSLLH.js');
let NavTIMEGPS = require('./NavTIMEGPS.js');
let NavSBAS = require('./NavSBAS.js');
let CfgNAV5 = require('./CfgNAV5.js');
let TimTM2 = require('./TimTM2.js');
let CfgGNSS_Block = require('./CfgGNSS_Block.js');
let MonHW6 = require('./MonHW6.js');
let RxmEPH = require('./RxmEPH.js');
let RxmRTCM = require('./RxmRTCM.js');
let RxmSVSI = require('./RxmSVSI.js');
let UpdSOS_Ack = require('./UpdSOS_Ack.js');
let CfgGNSS = require('./CfgGNSS.js');
let EsfMEAS = require('./EsfMEAS.js');
let RxmSFRBX = require('./RxmSFRBX.js');
let NavRELPOSNED = require('./NavRELPOSNED.js');
let NavSBAS_SV = require('./NavSBAS_SV.js');
let Inf = require('./Inf.js');
let MonHW = require('./MonHW.js');
let CfgHNR = require('./CfgHNR.js');
let EsfSTATUS_Sens = require('./EsfSTATUS_Sens.js');
let NavSOL = require('./NavSOL.js');
let NavDOP = require('./NavDOP.js');
let NavATT = require('./NavATT.js');
let CfgPRT = require('./CfgPRT.js');
let RxmRAWX_Meas = require('./RxmRAWX_Meas.js');
let NavVELNED = require('./NavVELNED.js');
let RxmRAWX = require('./RxmRAWX.js');
let CfgUSB = require('./CfgUSB.js');
let CfgANT = require('./CfgANT.js');
let MonGNSS = require('./MonGNSS.js');
let CfgINF = require('./CfgINF.js');
let HnrPVT = require('./HnrPVT.js');
let NavSAT_SV = require('./NavSAT_SV.js');
let EsfRAW = require('./EsfRAW.js');
let NavSAT = require('./NavSAT.js');
let EsfINS = require('./EsfINS.js');
let AidALM = require('./AidALM.js');
let NavSVINFO = require('./NavSVINFO.js');
let NavVELECEF = require('./NavVELECEF.js');
let CfgNMEA6 = require('./CfgNMEA6.js');
let CfgINF_Block = require('./CfgINF_Block.js');
let EsfRAW_Block = require('./EsfRAW_Block.js');
let NavSVINFO_SV = require('./NavSVINFO_SV.js');
let CfgDGNSS = require('./CfgDGNSS.js');
let AidHUI = require('./AidHUI.js');
let CfgNMEA = require('./CfgNMEA.js');
let NavDGPS = require('./NavDGPS.js');
let MonVER = require('./MonVER.js');
let RxmSVSI_SV = require('./RxmSVSI_SV.js');
let NavSTATUS = require('./NavSTATUS.js');
let CfgRST = require('./CfgRST.js');
let CfgMSG = require('./CfgMSG.js');
let NavPOSECEF = require('./NavPOSECEF.js');
let NavPVT = require('./NavPVT.js');
let RxmALM = require('./RxmALM.js');
let CfgNAVX5 = require('./CfgNAVX5.js');
let AidEPH = require('./AidEPH.js');
let RxmRAW = require('./RxmRAW.js');
let CfgDAT = require('./CfgDAT.js');
let CfgTMODE3 = require('./CfgTMODE3.js');
let Ack = require('./Ack.js');
let MgaGAL = require('./MgaGAL.js');
let RxmRAW_SV = require('./RxmRAW_SV.js');
let CfgRATE = require('./CfgRATE.js');
let CfgCFG = require('./CfgCFG.js');
let UpdSOS = require('./UpdSOS.js');
let NavSVIN = require('./NavSVIN.js');
let NavTIMEUTC = require('./NavTIMEUTC.js');
let MonVER_Extension = require('./MonVER_Extension.js');
let NavPVT7 = require('./NavPVT7.js');
let CfgNMEA7 = require('./CfgNMEA7.js');

module.exports = {
  NavCLOCK: NavCLOCK,
  RxmSFRB: RxmSFRB,
  CfgSBAS: CfgSBAS,
  EsfSTATUS: EsfSTATUS,
  NavDGPS_SV: NavDGPS_SV,
  NavPOSLLH: NavPOSLLH,
  NavTIMEGPS: NavTIMEGPS,
  NavSBAS: NavSBAS,
  CfgNAV5: CfgNAV5,
  TimTM2: TimTM2,
  CfgGNSS_Block: CfgGNSS_Block,
  MonHW6: MonHW6,
  RxmEPH: RxmEPH,
  RxmRTCM: RxmRTCM,
  RxmSVSI: RxmSVSI,
  UpdSOS_Ack: UpdSOS_Ack,
  CfgGNSS: CfgGNSS,
  EsfMEAS: EsfMEAS,
  RxmSFRBX: RxmSFRBX,
  NavRELPOSNED: NavRELPOSNED,
  NavSBAS_SV: NavSBAS_SV,
  Inf: Inf,
  MonHW: MonHW,
  CfgHNR: CfgHNR,
  EsfSTATUS_Sens: EsfSTATUS_Sens,
  NavSOL: NavSOL,
  NavDOP: NavDOP,
  NavATT: NavATT,
  CfgPRT: CfgPRT,
  RxmRAWX_Meas: RxmRAWX_Meas,
  NavVELNED: NavVELNED,
  RxmRAWX: RxmRAWX,
  CfgUSB: CfgUSB,
  CfgANT: CfgANT,
  MonGNSS: MonGNSS,
  CfgINF: CfgINF,
  HnrPVT: HnrPVT,
  NavSAT_SV: NavSAT_SV,
  EsfRAW: EsfRAW,
  NavSAT: NavSAT,
  EsfINS: EsfINS,
  AidALM: AidALM,
  NavSVINFO: NavSVINFO,
  NavVELECEF: NavVELECEF,
  CfgNMEA6: CfgNMEA6,
  CfgINF_Block: CfgINF_Block,
  EsfRAW_Block: EsfRAW_Block,
  NavSVINFO_SV: NavSVINFO_SV,
  CfgDGNSS: CfgDGNSS,
  AidHUI: AidHUI,
  CfgNMEA: CfgNMEA,
  NavDGPS: NavDGPS,
  MonVER: MonVER,
  RxmSVSI_SV: RxmSVSI_SV,
  NavSTATUS: NavSTATUS,
  CfgRST: CfgRST,
  CfgMSG: CfgMSG,
  NavPOSECEF: NavPOSECEF,
  NavPVT: NavPVT,
  RxmALM: RxmALM,
  CfgNAVX5: CfgNAVX5,
  AidEPH: AidEPH,
  RxmRAW: RxmRAW,
  CfgDAT: CfgDAT,
  CfgTMODE3: CfgTMODE3,
  Ack: Ack,
  MgaGAL: MgaGAL,
  RxmRAW_SV: RxmRAW_SV,
  CfgRATE: CfgRATE,
  CfgCFG: CfgCFG,
  UpdSOS: UpdSOS,
  NavSVIN: NavSVIN,
  NavTIMEUTC: NavTIMEUTC,
  MonVER_Extension: MonVER_Extension,
  NavPVT7: NavPVT7,
  CfgNMEA7: CfgNMEA7,
};

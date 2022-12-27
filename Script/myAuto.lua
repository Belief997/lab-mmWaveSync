------------------------------ CONFIGURATIONS ----------------------------------
-- Use "DCA1000" for working with DCA1000
capture_device  = "DCA1000"

-- SOP mode
SOP_mode        = 2

-- RS232 connection baud rate
baudrate        = 115200
-- RS232 COM Port number
uart_com_port   = 9
-- Timeout in ms
timeout         = 1000

-- BSS firmware
bss_path        = "C:\\ti\\mmwave_studio\\mmwave_studio_02_00_00_02\\rf_eval_firmware\\radarss\\xwr12xx_xwr14xx_radarss.bin"
-- MSS firmware
mss_path        = "C:\\ti\\mmwave_studio\\mmwave_studio_02_00_00_02\\rf_eval_firmware\\masterss\\xwr12xx_xwr14xx_masterss.bin"

adc_data_path   = "D:\\ti\\mmwave_studio_02_01_01_00\\mmWaveStudio\\PostProc\\adc_data.bin"


------------------------- Connect Tab settings ---------------------------------
-- ar1.FullReset()
-- RSTD.Sleep(5000)


-- -- Select Capture device
-- ret=ar1.SelectCaptureDevice(capture_device)
-- if(ret~=0)
-- then
--     print("******* Wrong Capture device *******")
--     return
-- end

-- -- SOP mode
-- ret=ar1.SOPControl(SOP_mode)
-- RSTD.Sleep(timeout)
-- if(ret~=0)
-- then
--     print("******* SOP FAIL *******")
--     return
-- end

-- -- RS232 Connect
-- ret=ar1.Connect(uart_com_port,baudrate,timeout)
-- RSTD.Sleep(timeout)
-- if(ret~=0)
-- then
--     print("******* Connect FAIL *******")
--     return
-- end

-- ar1.Calling_IsConnected()
-- ar1.SelectChipVersion("AR1243")
-- ar1.deviceVariantSelection("XWR1443")
-- ar1.frequencyBandSelection("77G")
-- ar1.SelectChipVersion("XWR1443")


-- -- Download BSS Firmware
-- ret=ar1.DownloadBSSFw(bss_path)
-- RSTD.Sleep(2*timeout)
-- if(ret~=0)
-- then
--     print("******* BSS Load FAIL *******")
--     return
-- end
-- -- 检查 BSS 版本号
-- ar1.GetBSSFwVersion()
-- -- BSSFwVersion:(02.00.00.01 (05/10/17))
-- ar1.GetBSSPatchFwVersion()
-- -- BSSPatchFwVersion:(01.02.00.03 (24/10/18))


-- -- Download MSS Firmware
-- ret=ar1.DownloadMSSFw(mss_path)
-- RSTD.Sleep(2*timeout)
-- if(ret~=0)
-- then
--     print("******* MSS Load FAIL *******")
--     return
-- end
-- -- 检查 MSS 版本号
-- ar1.GetMSSFwVersion()
-- -- MSSFwVersion:(01.02.00.02 (19/10/18))

-- -- SPI Connect
-- ar1.PowerOn(0, 1000, 0, 0)

-- -- RF Power UP
-- ar1.RfEnable()

------------------------- StaticConfig Tab settings ---------------------------------
-- ar1.ChanNAdcConfig(1, 1, 0, 1, 1, 1, 1, 2, 1, 0)
-- ar1.LPModConfig(0, 0)
-- -- freq limit ?
-- ar1.RfInit()

------------------------- DataConfig Tab settings ---------------------------------
-- ar1.DataPathConfig(513, 1216644150, 0)
-- ar1.LvdsClkConfig(1, 1)
-- ar1.LVDSLaneConfig(0, 1, 1, 1, 1, 1, 0, 0)



------------------------- SensorConfig Tab settings ---------------------------
-- ar1.ProfileConfig(0, 77, 100, 6, 60, 0, 0, 0, 0, 0, 0, 0.483, 0, 256, 10000, 0, 0, 30)
-- ar1.ChirpConfig(0, 0, 0, 0, 0, 0, 0, 1, 0, 0)
-- ar1.FrameConfig(0, 0, 32, 128, 40, 0, 0, 2)

-- ar1.DisableTestSource(0)
-- ar1.GetCaptureCardDllVersion()
-- ar1.SelectCaptureDevice("DCA1000")
-- ar1.CaptureCardConfig_EthInit("192.168.33.30", "192.168.33.180", "12:34:56:78:90:12", 4096, 4098)
-- ar1.CaptureCardConfig_Mode(1, 1, 1, 2, 3, 30)
-- ar1.CaptureCardConfig_PacketDelay(25)
-- ar1.GetCaptureCardFPGAVersion()

--Start Record ADC data
ar1.CaptureCardConfig_StartRecord(adc_data_path, 1)
RSTD.Sleep(1000)

--Trigger frame
ar1.StartFrame()
RSTD.Sleep(5000)


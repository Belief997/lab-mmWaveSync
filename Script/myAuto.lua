------------------------------ CONFIGURATIONS ----------------------------------
adc_data_path   = "D:\\ti\\mmwave_studio_02_01_01_00\\mmWaveStudio\\PostProc\\adc_data.bin"

-- ar1.ProfileConfig(0, 77, 100, 6, 60, 0, 0, 0, 0, 0, 0, 0.483, 0, 256, 10000, 0, 0, 30)
-- ar1.ChirpConfig(0, 0, 0, 0, 0, 0, 0, 1, 0, 0)


--Start Record ADC data
ar1.CaptureCardConfig_StartRecord(adc_data_path, 1)
RSTD.Sleep(1000)

--Trigger frame
ar1.StartFrame()
RSTD.Sleep(5000)


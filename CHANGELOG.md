## V1.0 (2026-05-25)
- before this version I already have coded serveral versions based on esp32S3.
  since its adc convert frequence not even reach 100kHz,I turn to esp32c5,early
  versions have on referanced value.
- this V1.0 has turned to esp32c5,but its code laying framework has been over
  designed.for emaple,if you want to know what a function in task layer actually do,
  the code needs mutiple jumps even more than six times.I really have no idea about
  how I thought before.
- commiting this absurd version just to witness how much this insignificant individual
  can improve in this project.

## V2.0 (2026-05-27)
- this project has been completely restructed,bsp layer provides the core function 
  interface to both upper layer and drv layer,and abandons drv register machanism 
  used in V1.0.  
- I employ a spi_encoder as5047p in this version,but it has many confusing problems as follow:
    1.experimentally measured its spi mode is 3,it works well in 10Mhz when and only when use mode3.
    2.based on previous point,it angle_read are very precise,but it err bit regularly return 1.
    3.with the first two items,the encoder read will fail since err bit verifying.
## V2.1 (2026-05-27)
- v2.0's encoder's return is invalid but "right"，this version I delete some verifying,and
  fixed spi mode to mode3 to let encoder work preferencially.
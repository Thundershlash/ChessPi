# -*- Mode: Python; indent-tabs-mode: t; c-basic-offset: 3; tab-width: 3 -*- #
# raio8870.py
# Copyright (C) 2017 Daniel Marquardt <thundershlash@gmx.net>
#
# c_berry_touch is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# c_berry_touch is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

from libbcm2835._bcm2835 import *
import time

class CBerryTouch():
   _MOSI       = 10 # RPI_V2_GPIO_P1_19
   _MISO       = 9  # RPI_V2_GPIO_P1_21
   _SCLK       = 11 # RPI_V2_GPIO_P1_23
   _OE         = 17 # RPI_V2_GPIO_P1_11
   _SPI_CE1    = 7  # RPI_V2_GPIO_P1_26
   _RAIO_RS    = 18 # RPI_V2_GPIO_P1_12
   _RAIO_RST   = 25 # RPI_V2_GPIO_P1_22
   _RAIO_CS    = 8  # RPI_V2_GPIO_P1_24
   _RAIO_WR    = 24 # RPI_V2_GPIO_P1_18
   _RAIO_RD    = 23 # RPI_V2_GPIO_P1_16
   _RAIO_WAIT  = 22 # RPI_V2_GPIO_P1_15
   _RAIO_INT   = 27 # RPI_V2_GPIO_P1_13
   
   # touch 
   _TOUCH_AVAILABLE = 1

   # color modes (color depths) { CM_4K=0, CM_65K }
   _COLOR_MODE = 'CM_65K'
   
   if _COLOR_MODE == 'CM_4K':
      _BankNo_WR=0
      _BankNo_RD=1

   # TFT dimensions
   _DISPLAY_WIDTH = 320
   _DISPLAY_HEIGHT = 240
   _PICTURE_PIXELS = _DISPLAY_WIDTH*_DISPLAY_HEIGHT

   # RAIO register -> see datasheet RAIO8870
   _PCOD = 0x00
   _PWRR = 0x01
   _MRWC = 0x02
   _PCLK = 0x04

   _SYSR = 0x10
   _DRGB = 0x11
   _IOCR = 0x12
   _IODR = 0x13

   _HDWR  = 0x14
   _HNDFTR = 0x15
   _HNDR  = 0x16
   _HSTR  = 0x17
   _HPWR  = 0x18

   _VDHR0 = 0x19
   _VDHR1 = 0x1a
   _VNDR0 = 0x1b
   _VNDR1 = 0x1c
   _VSTR0 = 0x1d
   _VSTR1 = 0x1e
   _VPWR  = 0x1f

   _DPCR  = 0x20
   _FNCR0 = 0x21
   _FNCR1 = 0x22
   _CGSR = 0x23
   _HOFS0 = 0x24
   _HOFS1 = 0x25
   _VOFS0 = 0x26
   _VOFS1 = 0x27
   _ROMS = 0x28

   _FLDR = 0x29

   _HSAW0 = 0x30
   _HSAW1 = 0x31
   _VSAW0 = 0x32
   _VSAW1 = 0x33
   _HEAW0 = 0x34
   _HEAW1 = 0x35
   _VEAW0 = 0x36
   _VEAW1 = 0x37
   _HSSW0 = 0x38
   _HSSW1 = 0x39
   _VSSW0 = 0x3a
   _VSSW1 = 0x3b
   _HESW0 = 0x3c
   _HESW1 = 0x3d
   _VESW0 = 0x3e
   _VESW1 = 0x3f

   _MWCR0 = 0x40
   _MWCR1 = 0x41
   _TFCR  = 0x42
   _TBCR  = 0x43
   _BTCR  = 0x44
   _CURS  = 0x45
   _CURH0 = 0x46
   _CURH1 = 0x47
   _CURV0 = 0x48
   _CURV1 = 0x49
   _RCURH0 = 0x4a
   _RCURH01 = 0x4b
   _RCURV0 = 0x4c
   _RCURV1 = 0x4d
   _MRCD  = 0x4e
   _BECR0 = 0x50
   _BECR1 = 0x51
   _LTPR0 = 0x52
   _LTPR1 = 0x53
   _HSBE0 = 0x54
   _HSBE1 = 0x55
   _VSBE0 = 0x56
   _VSBE1 = 0x57
   _HDBE0 = 0x58
   _HDBE1 = 0x59
   _VDBE0 = 0x5a
   _VDBE1 = 0x5b
   _BEWR0 = 0x5c
   _BEWR1 = 0x5d
   _BEHR0 = 0x5e
   _BEHR1 = 0x5f

   _BGCR0 = 0x60
   _BGCR1 = 0x61
   _BGCR2 = 0x62
   _FGCR0 = 0x63
   _FGCR1 = 0x64
   _FGCR2 = 0x65
   _PTNO  = 0x66
   _BGTR  = 0x67

   _TPCR0 = 0x70
   _TPCR1 = 0x71
   _TPXH  = 0x72
   _TPYH  = 0x73
   _TPXYL = 0x74

   _GCHP0 = 0x80
   _GCHP1 = 0x81
   _GCVP0 = 0x82
   _GCVP1 = 0x83
   _GCC0  = 0x84
   _GCC1  = 0x85

   _PLLC1 = 0x88
   _PLLC2 = 0x89

   _P1CR  = 0x8a
   _P1DCR = 0x8b
   _P2CR  = 0x8c
   _P2DCR = 0x8d
   _MCLR  = 0x8e
   _INTC  = 0x8f

   _DCR   = 0x90
   _DLHSR0 = 0x91
   _DLHSR1 = 0x92
   _DLVSR0 = 0x93
   _DLVSR1 = 0x94
   _DLHER0 = 0x95
   _DLHER1 = 0x96
   _DLVER0 = 0x97
   _DLVER1 = 0x98
   _DCHR0  = 0x99
   _DCHR1 = 0x9a
   _DCVR0 = 0x9b
   _DCVR1 = 0x9c
   _DCRR  = 0x9d

   _TCR1 = 0xa0
   _TCR2 = 0xa1
   _OEHTCR1 = 0xa2
   _OEHTCR2 = 0xa3
   _OEHTCR3 = 0xa4
   _OEHTCR4 = 0xa5
   _OEHTCR5 = 0xa6
   _OEHTCR6 = 0xa7
   _OEHTCR7 = 0xa8
   _OEHTCR8 = 0xa9

   _STHTCR1 = 0xaa
   _STHTCR2 = 0xab
   _STHTCR3 = 0xac
   _STHTCR4 = 0xad

   _Q1HCR1 = 0xae
   _Q1HCR2 = 0xaf

   _OEVTCR1 = 0xb0
   _OEVTCR2 = 0xb1
   _OEVTCR3 = 0xb2
   _OEVTCR4 = 0xb3
   _CKVTCR1 = 0xb4
   _CKVTCR2 = 0xb5
   _CKVTCR3 = 0xb6
   _CKVTCR4 = 0xb7
   _STVTCR1 = 0xb8
   _STVTCR2 = 0xb9
   _STVTCR3 = 0xba
   _STVTCR4 = 0xbb
   _STVTCR5 = 0xbc
   _STVTCR6 = 0xbd
   _STVTCR7 = 0xbe
   _STVTCR8 = 0xbf

   _COMTCR1 = 0xc0
   _COMTCR2 = 0xc1
   _RGBTCR1 = 0xc2
   _RGBTCR2 = 0xc3

   # colors "RRRGGGBB"
   _COLOR_RED     =  0xE0
   _COLOR_BLUE    =  0x03
   _COLOR_GREEN   =  0x1C
   _COLOR_BLACK   =  0x00
   _COLOR_WHITE   =  0xFF
   _COLOR_CYAN    =  0x1F 
   _COLOR_YELLOW  =  0xFC
   _COLOR_MAGENTA =  0xE3
   _COLOR_DARK_GREEN = 0x0C

   # ROP functions
   _ROP_SOURCE = 0xC

   # BTE operation functions
   _BTE_MOVE_POSITIVE =  0x02
   _BTE_SOLID_FILL   =   0x0C

   # enumeration of drawing modes
   _DRAW_MODES = {'CIRCLE_NONFILL','CIRCLE_FILL','SQUARE_NONFILL','SQUARE_FILL','LINE'}
   
   tc = 0x00, 0x00
   char_height = 15
   
   if _TOUCH_AVAILABLE == 1:   
      # definition of filter   size
      _DEBOUNCE_BUFFER_SIZE = 4
      low_pass_x = [0]
      low_pass_y = [0]
      
       # enumeration of touch modes    
      _TOUCH_FUNCTIONS = {'down','pressed','up','no_touch'} 
      # declaration of touch structure
      class touch():
         pass

      my_touch = touch()
   
   def __init__(self):
      self._COM_init_board()
      self._RAIO_init()

   # Initialization of Communication between RasPi and CBerry
   def _COM_init_board(self):
      if not bcm2835_init():
        return
      
   # set the pins to be an output and turn them on
      bcm2835_gpio_fsel( self._OE, BCM2835_GPIO_FSEL_OUTP )
      bcm2835_gpio_write( self._OE, HIGH )
   
      bcm2835_gpio_fsel( self._RAIO_RST, BCM2835_GPIO_FSEL_OUTP )
      bcm2835_gpio_write(self._RAIO_RST, HIGH )

      bcm2835_gpio_fsel( self._RAIO_CS, BCM2835_GPIO_FSEL_OUTP )
      bcm2835_gpio_write( self._RAIO_CS, HIGH )
      
      bcm2835_gpio_fsel( self._RAIO_RS, BCM2835_GPIO_FSEL_OUTP )
      bcm2835_gpio_write( self._RAIO_RS, HIGH )

      bcm2835_gpio_fsel( self._RAIO_WR, BCM2835_GPIO_FSEL_OUTP )
      bcm2835_gpio_write( self._RAIO_WR, HIGH )
   
      bcm2835_gpio_fsel( self._RAIO_RD, BCM2835_GPIO_FSEL_OUTP )
      bcm2835_gpio_write( self._RAIO_RD, HIGH )

      # now the inputs
      bcm2835_gpio_fsel( self._RAIO_WAIT, BCM2835_GPIO_FSEL_INPT )
      bcm2835_gpio_set_pud( self._RAIO_WAIT, BCM2835_GPIO_PUD_UP)
   
      bcm2835_gpio_fsel( self._RAIO_INT, BCM2835_GPIO_FSEL_INPT )
      bcm2835_gpio_set_pud( self._RAIO_INT, BCM2835_GPIO_PUD_UP)
   
      # bcm2835_gpio_fsel( _MISO, BCM2835_GPIO_FSEL_INPT )
      # bcm2835_gpio_set_pud( _MISO, BCM2835_GPIO_PUD_UP)
      
      # set pins for SPI
      # # bcm2835_gpio_fsel(MISO, BCM2835_GPIO_FSEL_ALT0); 
      # bcm2835_gpio_fsel(MOSI, BCM2835_GPIO_FSEL_ALT0)
      # bcm2835_gpio_fsel(SCLK, BCM2835_GPIO_FSEL_ALT0)
      # bcm2835_gpio_fsel(SPI_CE1, BCM2835_GPIO_FSEL_ALT0)
        
      # set the SPI CS register to the some sensible defaults
      # paddr = bcm2835_spi0 + BCM2835_SPI0_CS/8
      # bcm2835_peri_write( paddr, 0 ) # All 0s
    
      # clear TX and RX fifos
      # bcm2835_peri_write_nb( paddr, BCM2835_SPI0_CS_CLEAR )
      bcm2835_spi_begin() 
      bcm2835_spi_setBitOrder( BCM2835_SPI_BIT_ORDER_MSBFIRST )      
      bcm2835_spi_setDataMode( BCM2835_SPI_MODE0 )                 
      bcm2835_spi_setClockDivider( BCM2835_SPI_CLOCK_DIVIDER_2 ) 
      bcm2835_spi_chipSelect( BCM2835_SPI_CS1 )                      
      bcm2835_spi_setChipSelectPolarity( BCM2835_SPI_CS1, _LOW )


   # hard reset of the graphic controller and the tft
   def hard_reset(self):
      bcm2835_gpio_write( self._RAIO_RST, LOW )
      bcm2835_DelayMicroseconds( 10000 )
      bcm2835_gpio_write( self._RAIO_RST, HIGH )
      bcm2835_DelayMicroseconds( 1000 )
       
   # wait during raio is busy
   def _wait_for_raio(self):
      while bcm2835_gpio_lev( self.RAIO_WAIT ) == 0: 
         pass
         
   # write data via SPI to tft
   def _SPI_data_out( data ):
      tbuf[0] = hex(data)
      rbuf[0] = 0
      bcm2835_spi_transfernb( id(tbuf[0]), id(rbuf[0]), 1 ) 
      return rbuf[0]    

   # write byte to register
   def _RegWrite( self,reg ):
      bcm2835_gpio_write( self._RAIO_RS, HIGH )               
      bcm2835_gpio_write( self._RAIO_CS, LOW ) 
      bcm2835_gpio_write( self._RAIO_WR, LOW ) 
      bcm2835_gpio_write( self._OE, LOW )
       
      self._SPI_data_out( reg )
    
      bcm2835_gpio_write( self._RAIO_WR, HIGH )
      bcm2835_gpio_write( self._RAIO_CS, HIGH ) 
      bcm2835_gpio_write( self._OE, HIGH )

   # write byte to tft
   def _DataWrite( self,data ): 
      bcm2835_gpio_write( self._RAIO_RS, LOW ) 
      bcm2835_gpio_write( self._RAIO_CS, LOW ) 
      bcm2835_gpio_write( self._RAIO_WR, LOW ) 
      bcm2835_gpio_write( self._OE, LOW )
    
      self._SPI_data_out( data )
        
      bcm2835_gpio_write( self._RAIO_WR, HIGH )
      bcm2835_gpio_write( self._RAIO_CS, HIGH ) 
      bcm2835_gpio_write( self._OE, HIGH )
      
   def _DataRead(self):
      bcm2835_gpio_write( self._RAIO_RS, LOW ) 
      bcm2835_gpio_write( self._RAIO_CS, LOW ) 
      bcm2835_gpio_write( self._RAIO_WR, HIGH )
      bcm2835_gpio_write( self._RAIO_RD, LOW )
      bcm2835_gpio_write( self._OE, LOW )
    
      data = self._SPI_data_out( '0x00' )
        
      bcm2835_gpio_write( self._RAIO_WR, HIGH )
      bcm2835_gpio_write( self._RAIO_CS, HIGH ) 
      bcm2835_gpio_write( self._RAIO_RD, HIGH )
      bcm2835_gpio_write( self._OE, HIGH )
      return data
      
      
   # Initialization of RAIO8870
   def _RAIO_init(self):
      PLL_Initial_Flag = 0
    
      # PLL settings (System Clock)  
   
      if PLL_Initial_Flag == 0:           # wait until PLL is ready
         PLL_Initial_Flag = 1                # set Flag to avoid repeated PLL init
      
         self.RAIO_SetRegister( self,self._PLLC1, 0x07 )     # set sys_clk 
         bcm2835_DelayMicroseconds( 200 )
         self.RAIO_SetRegister( self,self._PLLC2, 0x03 )     # set sys_clk 
         bcm2835_delayMicroseconds( 200 )
      
         self.RAIO_SetRegister( self,self._PWRR, 0x01 )     # Raio software reset ( bit 0 ) set
         self.RAIO_SetRegister( self,self._PWRR, 0x00 )     # Raio software reset ( bit 0 ) set to 0
         time.sleep(.100) 


      # color modes (color depths)  
   
         if self._COLOR_MODE == 'CM_65K': 
         # System Configuration Register
            self.RAIO_SetRegister( self,self._SYSR, 0x0A )    # digital TFT
                                            # parallel data out
                                            # no external memory
                                            # 8bit memory data bus
                                            # 16bpp 65K color
                                            # 16bit MCU-interface (data)
            self.RAIO_SetRegister( self,self._DPCR, 0x00 )    # one layer   
         elif _COLOR_MODE == 'CM_4K':
            # System Configuration Register
            self.RAIO_SetRegister( self,self._SYSR, 0x06 )    # digital TFT
                                            # parallel data out
                                            # no external memory
                                            # 8bit memory data bus
                                            # 12bpp 4K color
                                            # 16bit MCU-interface (data)
            self.RAIO_SetRegister( self,self._DPCR, 0x80 )    # two layers   
            self.RAIO_SetRegister( self,self._MWCR1, self.BankNo_WR )
            self.RAIO_SetRegister( self,self._LTPR0, self.BankNo_RD )                      
 
      # horizontal settings
      # 0x27+1 * 8 = 320 pixel  
      self.RAIO_SetRegister( self,self._HDWR , (self._DISPLAY_WIDTH / 8) - 1 )
      # Horizontal Non-Display Period Fine Tuning   
      self.RAIO_SetRegister( self,self._HNDFTR, 0x02 )
    
      # HNDR , Horizontal Non-Display Period Bit[4:0] 
      # Horizontal Non-Display Period (pixels) = (HNDR + 1)*8    
      self.RAIO_SetRegister( self,self._HNDR, 0x03 )
      # HSTR , HSYNC Start Position[4:0], HSYNC Start Position(PCLK) = (HSTR + 1)*8     0x02    
      self.RAIO_SetRegister( self,self._HSTR, 0x04 )                                 

      # HPWR , HSYNC Polarity ,The period width of HSYNC. 
      # 1xxxxxxx activ high 0xxxxxxx activ low
      # HSYNC Width [4:0] HSYNC Pulse width
      # (PCLK) = (HPWR + 1)*8
      self.RAIO_SetRegister( self,self._HPWR, 0x03 )
    
    
      # vertical settings    
      # 0x0EF +1 = 240 pixel
      self.RAIO_SetRegister( self,self._VDHR0 , ( (self._DISPLAY_HEIGHT-1) & 0xFF ) ) 
      self.RAIO_SetRegister(  self,self._VDHR1 , ( (self._DISPLAY_HEIGHT-1) >> 8)    )
    
      # VNDR0 , Vertical Non-Display Period Bit [7:0]
      # Vertical Non-Display area = (VNDR + 1)
      # VNDR1 , Vertical Non-Display Period Bit [8]
      # Vertical Non-Display area = (VNDR + 1)              
      self.RAIO_SetRegister( self,self._VNDR0, 0x10 )
      self.RAIO_SetRegister( self,self._VNDR1, 0x00 )
                      
      # VPWR , VSYNC Polarity ,VSYNC Pulse Width[6:0]
      # VSYNC , Pulse Width(PCLK) = (VPWR + 1) 
      self.RAIO_SetRegister( self,self._VPWR, 0x00 )
      
      # miscellaneous settings 
    
      # active Window
      self.Active_Window(self, 0, self._DISPLAY_WIDTH-1, 0, self._DISPLAY_HEIGHT-1 )     
        
      # PCLK fetch data on rising edge 
      self.RAIO_SetRegister(self, self._PCLK, 0x00 )   

      # Backlight dimming       
      self.RAIO_SetBacklightPWMValue(self,50)

      # memory clear with background color
      self.Text_Background_Color( self,self._COLOR_WHITE )                
      self.RAIO_SetRegister( self,self._MCLR, 0x81 )     
      self.TFT_wait_for_raio(self) 
  
      self.RAIO_SetRegister( self,self._IODR, 0x07 )    
      self.RAIO_SetRegister( self,self._PWRR, 0x80 )
   
      if self._TOUCH_AVAILABLE == 1:
         # Touch Panel enable
         # wait 4096 system clocks period
         # ADC clock = system clock / 16
         self.RAIO_SetRegister( self,self._TPCR0, 0xB7 )   
         # 4wire, auto mode, internal vref enabled
         # debounce enabled, idle mode
         self.RAIO_SetRegister( self,self._TPCR1, 0x84 )
   
         # enable touch interrupt
         self.RAIO_SetRegister( self,self._INTC, 0x40 ) 

         ################### hier gehts weiter
         # init touch structure
         self.my_touch.state = no_touch
   
         # init touch values
         self.touch_buffer_full = 0
         self.low_pass_pointer = 0       

         
   # Read data from a register
   def RAIO_GetRegister(self,reg):
      self._RegWrite(self,reg)
      value = ReadData(self)
      return value

   # write command to a register
   def RAIO_SetRegister( self,reg, value ):
      _RegWrite(self,reg)
      _DataWrite(self,value)

   # set PWM value for backlight -> 0 (0% PWM) - 255 (100% PWM)
   def RAIO_SetBacklightPWMValue( self,BL_value ):
      # Enable PWM1 output devider 256  
      self.RAIO_SetRegister(self, self._P1CR, 0x88) 
      # BL_vaue = 0 (0% PWM) - 255 (100% PWM)
      self.RAIO_SetRegister(self, self._P1DCR, BL_value )

   # set coordinates for active window
   # p1, p2: tupel(x,y)
   def Active_Window( self,p1, p2):
      # Set p1
      self.RAIO_SetRegister( self,self._HSAW0, i16_split(p1[0],low) )
      self.RAIO_SetRegister( self,self._HSAW1, i16_split(p1[0],high) )
      self.RAIO_SetRegister( self,self._VEAW0, i16_split(p1[1],low) )
      self.RAIO_SetRegister( self,self._VEAW1, i16_split(p1[1],high) )
      
      # Set p2
      self.RAIO_SetRegister( self,self._HEAW0, i16_split(p2[0],low) )
      self.RAIO_SetRegister( self,self._HEAW1, i16_split(p2[0],high) )
      self.RAIO_SetRegister( self,self._VSAW0, i16_split(p2[1],low) )
      self.RAIO_SetRegister( self,self._VSAW1, i16_split(p2[1],high) )
      

   # set cursor 
   # cur: tupel(x,y)
   def RAIO_set_cursor( self,cur ):
      self.RAIO_SetRegister( self,self._CURH0, i16_split(cur[0],low) )
      self.RAIO_SetRegister( self,self._CURH1, i16_split(cur[0],high) )
   
      self.RAIO_SetRegister( self,self._CURV0, i16_split(cur[1],low) )
      self.RAIO_SetRegister( self,self._CURV1, i16_split(cur[1],high) )
      

   # set mode for BET (Block Transfer Engine)
   def BTE_mode( self,bte_operation, rop_function ):
      self.RAIO_SetRegister(self,self._BECR1, bte_operation | (rop_function<<4))

   # set color -> see color defines
   def Text_Background_Color( self,color ):
      self.RAIO_SetRegister( self,self._TBCR, color )
      
      
   def Text_Foreground_Color( color ):
      self.RAIO_SetRegister( self,self._TFCR, color)


   # clear memory
   def RAIO_clear_screen():
      self.RAIO_SetRegister( self,self._MCLR , 0x81 ) 
      self._wait_for_raio(self)


   # set coordinates for drawing
   # p1, p2: Tupel(x,y)
   def Set_Geometric_Coordinate(self, p1, p2 ):
      # P1, x
      self.RAIO_SetRegister( self,self._DLHSR0, i16_split(p1[0],low) )
      self.RAIO_SetRegister( self,self._DLHSR1, i16_split(p1[0],high) )
      # P1, y
      self.RAIO_SetRegister( self,self._DLVSR0, i16_split(p1[1],low)  )
      self.RAIO_SetRegister( self,self._DLVSR1, i16_split(p1[1],high) )

      # P2, x
      self.RAIO_SetRegister( self,self._DLHER0, i16_split(p2[0],low) )
      self.RAIO_SetRegister( self,self._DLHER1, i16_split(p2[0],high) )

      # P2, y
      self.RAIO_SetRegister( self,self._DLVER0, i16_split(p2[1],low) )
      self.RAIO_SetRegister( self,self._DLVER1, i16_split(p2[1],high) )

   def Set_Geometric_Coordinate_circle ( p1, rad ):
      # P1, x
      self.RAIO_SetRegister( self,self._DCHR0, i16_split(p1[0],low) )
      self.RAIO_SetRegister( self,self._DCHR1, i16_split(p1[0],high) )
   
      # P1, y
      self.RAIO_SetRegister( self,self._DCVR0, i16_split(p1[1],low) )
      self.RAIO_SetRegister( self,self._DCVR1, i16_split(p1[1],high) )
      
      # rad
      self.RAIO_SetRegister( self,self._DCRR, rad )

   # show the BMP picture on the TFT screen 
   def RAIO_Write_Picture( self,data, count ): # has to be checked how pointer works in python
      pass


   # set draw mode -> see DRAW_MODES
   def RAIO_StartDrawing(self,whattodraw ):
      if whattodraw == 'CIRCLE_NONFILL':
          self.RAIO_SetRegister( self,self._DCR,  0x40 )
      elif whattodraw == 'CIRCLE_FILL':
          self.RAIO_SetRegister( self,self._DCR,  0x60 )
      elif whattodraw == 'SQUARE_NONFILL':
          self.RAIO_SetRegister( self,self._DCR,  0x90 )
      elif whattodraw == 'SQUARE_FILL':
          self.RAIO_SetRegister( self,self._DCR,  0xB0 )
      elif whattodraw == 'LINE':
          self.RAIO_SetRegister( self,self._DCR,  0x80 )

   # draw some basic geometrical forms
   def Draw_Line( self,p1, p2 ):
      self.Set_Geometric_Coordinate( self,p1, p2 )
      self.RAIO_StartDrawing( self,'LINE' )

   def Draw_Square(self, p1, p2 ):
      self.Set_Geometric_Coordinate( self,p1, p22 )
      self.RAIO_StartDrawing(self, 'SQUARE_NONFILL' )
   
   def Draw_Circle( self,p1, rad ):
      self.Set_Geometric_Coordinate_circle (self, p1, rad )
      self.RAIO_StartDrawing( self,'CIRCLE_NONFILL' )

   # print text
   def RAIO_print_text(self,p1, text, BG_color, FG_color ):
      # set cursor
      self.RAIO_set_cursor( self,p1 )
   
      # set color 
      self.Text_Background_Color(self, BG_color )
      self.Text_Foreground_Color(self, FG_color )
   
      # set text mode
      self.RAIO_SetRegister( self,self._MWCR0, 0x80 )
   
      # write text to display
      self._RegWrite( self,self._MRWC )
   
      for i in xrange(0,len(text)-1):
         self._DataWrite( self,text[i] )
         self._wait_for_raio(self)
   
      self._wait_for_raio(self)
         
      # set graphic mode
      self.RAIO_SetRegister( self,self._MWCR0, 0x00 )


   # set font size
   def RAIO_SetFontSizeFactor(self, size ):
      size = (size & 0x0f)
      self.RAIO_SetRegister ( self,self._FNCR1, size )
      
   def i16_split(value, byte):
      if value > 4095: 
         high = int(hex(value)[:4],16)
         low = int('0x'+hex(value)[4:],16)
      elif value > 255:
         high = int('0x0'+hex(value)[2:3],16)
         low = int('0x'+hex(value)[3:],16)
      elif value <= 255:
         high = 0x00
         low = value
         
      if byte == 'high':
         return high
      elif byte == 'low':
         return low

   if _TOUCH_AVAILABLE == 1:     
   # get touch coordinates
      def RAIO_gettouch(self):
         mask= self.RAIO_GetRegister( self,self._INTC )
         touch = 0,0
   
         if mask & 0x04:
            # read the data for x and y
            touch[0] = self.RAIO_GetRegister ( self,self._TPXH )
            touch[1] = self.RAIO_GetRegister ( self,self._TPYH )
      
            # fill low pass filter with the new values
            low_pass_x.append(touch[0])
            low_pass_y.append(touch[1])
            low_pass_pointer = low_pass_pointer + 1
      
            if low_pass_pointer == self._DEBOUNCE_BUFFER_SIZE:
               low_pass_pointer = 0
               touch_buffer_full = 1
               
      
            # calculate the average
            my_touch.touch[0] = (low_pass_x[0] + low_pass_x[1] + low_pass_x[2] + low_pass_x[3] ) >> 2
            my_touch.touch[1] = (low_pass_y[0] + low_pass_y[1] + low_pass_y[2] + low_pass_y[3] ) >> 2

      
            if (touch_buffer_full == 1):
               if my_touch.state == 'down':
                  my_touch.state= 'pressed'
               elif my_touch.state == 'no_touch':
                  my_touch.state = 'down'
               else:
                  pass
      
            # clear touch irq
            mask = mask & 0xf4
            self.RAIO_SetRegister( self,self._INTC, mask ) 
   
            return  1
             
         else:
            if my_touch.state == 'up':
               my_touch.state = 'no_touch'
            elif  my_touch.state == 'pressed':
               my_touch.state = 'up'
            else:
               pass
      
            low_pass_pointer = 0
            touch_buffer_full = 0
            return 0    

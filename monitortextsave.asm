
	.include "basic.asm"

; put the IRQ and MNI code in RAM so that it can be changed

IRQ_vec	= VEC_SV+2		; IRQ code vector
NMI_vec	= IRQ_vec+$0A	; NMI code vector


;6522 VIA PORT A/B

PORTA = $A011 ;port A
PORTB = $A010 ;port B
DDRA  = $A013 ;data direction register for port A
DDRB  = $A012 ;data direction register for port B

SR   = $A01a ;shift regiter
ACR   = $A01b ;auxiliary control register
PCR   = $A01c ;peripheral control register
IFR   = $A01d ;interupt flag register
IER   = $A01e ;interupt enable register
T1CL = $A014  ;timer one counter low
T1CH = $A015 ;timer one counter high






DATA_READ  	= %00000011 ;Read data from memory array beginning at selected address
DATA_WRITE	= %00000010	;Write data to memory array beginning at selected address
WRDI	 	  	= %00000100	;Reset the write enable latch (disable write operations)	
WREN	 	  	= %00000110	;Set the write enable latch (enable write operations)
RDSR	 	  	= %00000101	;Read STATUS register
WRSR	 	  	= %00000100	;Write STATUS register



SCK            = %00000001; SPI clock
MOSI           = %00000010; SPI data out
MISO           = %01000000; SPI data in
CS1            = %00000100; Chip select 1
CS2            = %00001000; Chip select 2
CSH            = %00001100; Chip select 1 and 2 

;6551 ACIA SERIAL PORT 
ACIA_DATA         = $A020
ACIA_STAT         = $A021
ACIA_COMMAND      = $A022
ACIA_CONTROL      = $A023

save_a 	= $03f0
save_x 	= save_a+1
save_y 	= save_x+1
line   	= save_y+1
text_x 	= line+1 
kb_wptr 	= text_x+1         ;keyboad write pointer
kb_rptr 	= kb_wptr+1         ;keyboad read pointer
kb_flags 	= kb_rptr+1        ;keyboad flags

;ZERO PAGE VARIABLES

ADSAVELO =$E2
ADSAVEHI =$E3

SFLAG    = $E4
ADLO1    = kb_flags+1
ADHI1 	= ADLO1+1
ADLO 		= ADHI1+1
ADHI 		= ADLO+1
SAE  	   = ADHI+1 	
SAX  	 =SAE+1
SAY    =SAX+1

IRQSAVEA = $E5
IRQSAVEX = $E6
IRQSAVEY = $E7

SPIIN  	= $E8               ;SPI input
SPIOUT 	= $E9               ;SPI output
text_c 	= $EA               ;cursor position
c_x    	= $EB		; cursor x
c_y	   = $EC		; cursor y
blink  	= $ED               ;cursor blink

ticks     = $0400              ;timer 









RELEASE = %00000001
SHIFT   = %00000010
CAPS    = %00000100
kb_buffer = $0500          ;256-byte kb buffer 0500-05ff



	*=	$f100	
	


cls:                     ;clear the screen memory
  pha
  txa
  pha
    
  lda #$80
  sta text_c+1;
  lda #$00
  sta text_c;
  sta line
  lda #$01
  sta text_x
  
  ldx #$00
  lda #$00
clear:  
  sta	$8000,x
  sta	$8040,x
  sta	$8080,x
  sta	$80c0,x
  sta	$8100,x
  sta	$8140,x
  sta	$8180,x
  sta	$81c0,x
  sta	$8200,x
  sta	$8240,x
  sta	$8280,x
  sta	$82c0,x
  sta	$8300,x
  sta	$8340,x
  sta	$8380,x
  sta	$83c0,x
  sta	$8400,x
  sta	$8440,x
  sta	$8480,x
  sta	$84c0,x
  sta	$8500,x
  sta	$8540,x
  sta	$8580,x
  sta	$85c0,x
  sta	$8600,x
  sta	$8640,x
  sta	$8680,x
  sta	$86c0,x
  sta	$8700,x
  sta	$8740,x
  sta	$8780,x
  sta	$87c0,x
  inx
  cpx #$33
  beq exit_cls
  jmp clear
exit_cls:  
  pla
  tax
  pla
  rts	
	
	
write:
  STA save_a
  STY save_y
  STX save_x
  lda line
  cmp #$1f
  beq endpage
endpage_continue:
  lda save_a
  cmp #$0a           ; enter - go to second line
  beq write_end
  cmp #$0d           ; enter - go to second line
  beq enter_pressed
  cmp #$08           ; backspace - go to second line
  beq backspace_pressed
 
  ldy text_x
  cpy #$31
  bcc no_line_end
  ldy #$01
  pha 
  lda text_c
  adc #$40
  sta text_c
  lda text_c+1
  adc #$00
  sta text_c+1
  dec text_c
  pla 
  
no_line_end:  
  sta (text_c),y
  iny
  sty text_x
  
  
write_end:
  LDA save_a
  LDY save_y
  LDX save_x
  
  
  rts 
  
  
  
backspace_pressed: 
  pha
  ldy text_x
  dey
  sty text_x
  ldy text_x
  lda #$00
  sta (text_c),y
  iny
  sta (text_c),y
  pla
  jmp write_end
  
 
enter_pressed:
  pha 
  inc line
  ldx line
  ldy text_x
  lda #$00
  sta (text_c),y
  ldy #$01
  sty text_x
  
  lda text_c
  adc #$40
  sta text_c
  lda text_c+1
  adc #$00
  sta text_c+1
  dec text_c
  pla 
  jmp write_end
  

endpage:
  ldx #$00
mov1:
  lda	$8040,x
  sta	$8000,x
  
  lda $8080,x
  sta	$8040,x
  
  lda	$80c0,x
  sta	$8080,x
  
  lda	$8100,x
  sta	$80c0,x
  
  lda $8140,x
  sta	$8100,x
  
  lda	$8180,x
  sta	$8140,x
  
  lda	$81c0,x
  sta	$8180,x
  
  lda	$8200,x
  sta	$81c0,x
  
  lda	$8240,x
  sta	$8200,x
  inx
  cpx #$33
  bne mov1
  ldx #$00
mov2:  
  lda	$8280,x
  sta	$8240,x
  
  lda	$82c0,x
  sta	$8280,x
  
  lda	$8300,x
  sta	$82c0,x
  
  lda	$8340,x
  sta	$8300,x
  
  lda	$8380,x
  sta	$8340,x
  
  lda	$83c0,x
  sta	$8380,x
  
  lda	$8400,x
  sta	$83c0,x
  
  lda	$8440,x
  sta	$8400,x
  
  lda	$8480,x
  sta	$8440,x
  
  lda	$84c0,x
  sta	$8480,x
  
  lda	$8500,x
  sta	$84c0,x
  inx
  cpx #$33
  bne mov2  
  ldx #$00
mov3:   
  lda	$8540,x
  sta	$8500,x
  
  lda	$8580,x
  sta	$8540,x
  
  lda	$85c0,x
  sta	$8580,x
  
  lda	$8600,x
  sta	$85c0,x
  
  lda	$8640,x
  sta	$8600,x
  
  lda $8680,x
  sta	$8640,x
  
  lda	$86c0,x
  sta	$8680,x
  
  lda	$8700,x
  sta	$86c0,x
  
  lda	$8740,x
  sta	$8700,x
  
  lda	$8780,x
  sta	$8740,x
 
 
  lda	$87c0,x
  sta	$8780,x
 
  lda #$00
  sta	$87c0,x
  inx
  cpx #$33
  bne mov3
  dec line
  
  lda #$c0
  sta text_c;
  lda #$87
  sta text_c+1;
  lda #$01
  sta text_x
  
  jmp endpage_continue
 
 
irq:       
  sta IRQSAVEA
  stx IRQSAVEX
  sty IRQSAVEY
  
               ;                 7           6       5        4    3       2              1     0
  ;interrupt flag register: SEt/Clear | Timer 1 | Timer 2 | CB1 | CB2 | Shift register | CA1 | Ca2 |  
 lda IFR
 ASL A 
 bmi timer1_irq;timer1
 
 ASL A ;timer2
 ASL A ;CB1
 ASL A ;CB2
 ASL A ;SR
 ASL A ;CA1
 bmi keyboard_interrupt; CA1
 ASL A  ;CA2
 jmp exit;
   
timer1_irq:
  
  bit T1CL
  inc ticks
  bne end_timer
  inc ticks+1
  bne end_timer;
  inc ticks+2
  bne end_timer;
  inc ticks+3
end_timer:
  jmp exit  

keyboard_interrupt:
  

  lda kb_flags
  and #RELEASE   ; check if we're releasing a key
  beq read_key   ; otherwise, read the key

  lda kb_flags
  eor #RELEASE   ; flip the releasing bit
  sta kb_flags
  lda PORTA      ; read key value that's being released
  cmp #$12       ; left shift
  beq shift_up
  cmp #$59       ; right shift
  beq shift_up
  jmp exit


  
shift_up:
  lda kb_flags
  eor #SHIFT  ; flip the shift bit
  sta kb_flags
  jmp exit

read_key:
  lda PORTA
  cmp #$f0        ; if releasing a key
  beq key_release ; set the releasing bit
  cmp #$12        ; left shift
  beq shift_down
  cmp #$59        ; right shift
  beq shift_down
  
  cmp #$58
  beq caps_lock
  tax
  lda kb_flags
  and #SHIFT
  bne shifted_key
  
  lda kb_flags
  and #CAPS
  bne capsed_key
  
  lda keymap, x   ; map to character code
  jmp push_key

capsed_key:
  lda keymap_caps, x   ; map to character code
  jmp push_key

shifted_key:
  lda keymap_shifted, x   ; map to character code

push_key:
  ldx kb_wptr
  sta kb_buffer, x
  inc kb_wptr
  jmp exit

shift_down:
  lda kb_flags
  ora #SHIFT
  sta kb_flags
  jmp exit

key_release:
  lda kb_flags
  ora #RELEASE
  sta kb_flags

 
exit:
  lda IRQSAVEA
  ldx IRQSAVEX
  ldy IRQSAVEY
  
  rti
  
caps_lock:
  lda kb_flags
  eor #CAPS  ; flip the shift bit
  sta kb_flags
  jmp exit
	
  JMP END1	
	
;========================================================
; SPI INTERFACE ON ON PORT B OF THE VIA
;
;========================================================
	
	
	
spi_transceive:
   php
	sta SPIOUT ;store the output data;
	lda #0     
	sta SPIIN ;clear the input buffer
	ldy #8;   ;initialize the bit counter
	lda #MOSI ;set a register to MOSI bit mask

spi_loop:	
   asl SPIOUT ; move MSB of output into carry flag; next bit to send
   bcs spi_1  ;test carry flag
   
   ;MSB was 0, set mosi low
   BIT PORTB
	PHP
	PHA
	EOR #$FF
	AND PORTB
	STA PORTB
	PLA
	PLP
    
   jmp spi_2
spi_1:
   ;tsb PORTB  ;MSB was 1 set mosi high
   BIT PORTB
	PHP
	PHA
	ORA PORTB
	STA PORTB
	PLA
	PLP
	
spi_2:
   inc PORTB ;set clock high
   bit PORTB ;put MISO into overflow flag
   clc
   bvc spi_3 ;test overflow flag
   sec       ;overflow was set, set carry   
   
spi_3:
   rol SPIIN ; rotate carry flag into SPIIN buffer
   dec PORTB ; set clock low
   dey       ; decrement bit counter 
   bne spi_loop
   lda SPIIN
   clc
   plp
	rts


;========================================================
;SERIAL EEPROM  DRIVERS
;========================================================



writeenable:   
	lda #CS2 					;selecting EEPROM 1
	sta PORTB
	lda #WREN					;set write enable latch
   jsr spi_transceive
   lda #CSH					;set both EEprom CS high
	sta PORTB
   rts
   
writedisable:   
	lda #CS2 					;selecting EEPROM 1
	sta PORTB
	lda #WRDI					;reset write enable latch
   jsr spi_transceive
   lda #CSH					;set both EEprom CS high
	sta PORTB
   rts



END1:
;========================================================
;6502 Computer BIOS 
;	PROGRAM START 
;========================================================
RES_vec            ; reset vector point here`
	CLD				; clear decimal mode
	LDX	#$FF		; empty stack
	TXS				; set the stack

; set up vectors and interrupt code, copy them to page 2
;========================================================= 
;VIA SETUP/RESET 
;========================================================= 
  lda #$01
  sta PCR
  lda #%11000010  ;interrupt enable register: SEt/Clear | Timer 1 | Timer 2 | CB1 | CB2 | Shift register | CA1 | Ca2 |   0: interrupt disabled 1:interrupt enabled
  sta IER
  cli                       
                           ;|Timer one | Timer one |Timer 2|SR|SR|SR|PB|PA"|
  lda #%01011011 				;Enable Shift Register
  sta ACR
  
  lda #%00000000 				;Set all pins on port A to input
  sta DDRA
  
  
  sta SFLAG ; set clear flag to 0
  
  
  
  ;SPI INIT
  lda #$FF				;
  sta DDRB
  lda #$00				;
  sta PORTB
  lda #%00001111            ;nc/miso/nc/nc/cs2/cs1/mosi/SCK
  sta DDRB                  ;set input output pins on port b
  lda #$00
  sta SPIIN
  sta SPIOUT
  sta PORTB
  lda #CSH ;set cs1 and cs2 high
  sta PORTB
  
  ;SPI CS active low. clock active low
  
  
  lda #$0 
  sta SR
  lda #$2
  sta SR

  
init_timer:
 lda #$0
 sta ticks
 sta ticks+1
 sta ticks+2
 sta ticks+3
 lda #$50
 sta T1CL
 lda #$1e
 sta T1CH
 cli
 
  
 ;========================================================= 
 ;ACIA RESET 
 ;========================================================= 
  lda     #$00
  sta     ACIA_STAT
 ;          76543210   
  lda     #$B                          ;    -> no recv irq
  sta     ACIA_COMMAND
  ;         76543210
  lda     #$1F                        ;0001 11111 19200 baud, 8+1 
  sta     ACIA_CONTROL
 ;ACIA RESET END
 ;======================================================== 


  lda #$00
  sta kb_flags
  sta kb_wptr
  sta kb_rptr 
  jsr cls ;clear video memory 


  
	LDY	#END_CODE-LAB_vec	; set index/count
LAB_stlp:
	LDA	LAB_vec-1,Y		; get byte from interrupt code
	STA	VEC_IN-1,Y		; save to RAM
	DEY				; decrement index/count
	BNE	LAB_stlp		; loop if more to do

   lda #$0
   tay
   
LAB_signon:
	LDA	LAB_mess,Y		; get byte from sign on message
	BEQ	LAB_signon2		; exit loop if done
	JSR	V_OUTP		; output character
	INY				; increment index
	BNE	LAB_signon		; loop, branch always

  
LAB_signon2:
   lda #$0
   tay
LAB_signon2C:
	LDA	LAB_mess2,Y		; get byte from sign on message
	BEQ	start		; exit loop if done
	JSR	V_OUTP		; output character
	INY				; increment index
	BNE	LAB_signon2C		; loop, branch always

start:
	JMP	LAB_COLD		; do EhBASIC cold start


;================================================================
;BIOS routines
;================================================================


ACIAout
   pha                   ; save register a
   sta SAE
   stx SAX
   sty SAY
   
   lda #$2
   cmp SFLAG
   beq to_eprom
   
uart_out1:     
   lda ACIA_STAT         ; serial port status
   and #$10              ; is tx buffer empty $10
   beq uart_out1         ; no
   pla                   ; get char
   sta ACIA_DATA         ; Write character to Port
   jsr write
   RTS



to_eprom:
   
   ldy #$01
   pla
   sta (ADSAVELO),y
   jsr write
   inc ADSAVELO
   bne cont
   inc ADSAVEHI
cont:
   inc ADLO
   bne end_save
   inc ADHI

 end_save:
   lda SAE
   ldy SAY
   ldx SAX
	RTS


cursor:
  lda #00
  sta blink
  inc blink+1
  lda blink+1
  cmp #$01
  beq noshow
  cmp #$7f
  beq show
  jmp endcursor
  
show:  
  lda #$5f
  ldy text_x
  sta (text_c),y
  jmp endcursor
  
noshow:  
  lda #$00
  ldy text_x
  sta (text_c),y
  
endcursor:
  rts



ACIAin
  sta save_a
  stx save_x
  sty save_y  

   lda #$1
   cmp SFLAG
   beq load_eprom
  
  inc blink
  lda blink
  cmp #55
  bne here
  jsr cursor

here:
  lda save_a
  ldx save_x
  ldy save_y

  sei
  lda kb_rptr
  cmp kb_wptr
  cli
  BEQ nokeypressed		; branch if no byte waiting

  stx $305
  ldx kb_rptr
  lda kb_buffer, x
  sta $308
  inc kb_rptr
  ldx $305
  SEC			; flag byte received
  inc $309
  RTS
  
nokeypressed:
  lda ACIA_STAT        ; Serial port status
  and #$08             ; mask rcvr full bit
  BEQ	LAB_nobyw		    ; branch if no byte waiting
  lda ACIA_DATA        ; get char
  SEC				; flag byte received
  RTS
LAB_nobyw
	CLC				; flag no byte received
	RTS
	
load_eprom:
   lda ADLO1           ;addr0
   cmp ADLO            ;addr1
   lda ADHI1           ;addr0+1
   sbc ADHI            ;addr1+1
   
   bcc  not_done_loading ;addr1 isBigger
   ;bne ;addr0IsBigger
   ; else equal...
   lda #$0
   sta SFLAG
   lda #CSH
	sta PORTB
	lda #$0
   sta SFLAG
   CLC
   RTS
   
not_done_loading:   
   lda #00
   jsr spi_transceive ;bytes     
   
   SEC				; flag byte received
   inc ADLO1
   bne end_load
   inc ADHI1
 end_load:   
   RTS	
	
	
	
no_load:
   lda #$1
   sta SFLAG
   
   lda #0
   sta ADLO1
   sta ADHI1
   
   lda #CS2
	sta PORTB
	lda #DATA_READ
	jsr spi_transceive
	
	lda #00
   jsr spi_transceive ;adress 2
   lda #00
	jsr spi_transceive ;adress 1     
	
   lda #00
   jsr spi_transceive ;bytes     
   sta ADLO
   
   lda #00
   jsr spi_transceive ;bytes     
   sta ADHI
   
   
   lda #$0d  
   jsr write
   lda #"L"
   jsr write
   lda #"O"
   jsr write
   lda #"A"
   jsr write
   lda #"D"
   jsr write
   lda #$0d  
   jsr write
   RTS
   
DELAY:
   ldy #$FF
delay1:
   ldx #$FF
delay2:
   dex
   bne delay2
   dey
   bne delay1
	rts

no_save:

   LDA #$02
   sta SFLAG


  lda #$00
  sta ADSAVELO
  lda #$50
  sta ADSAVEHI
    
   lda #$00
   sta ADHI
   lda #$02
   sta ADLO
   
   lda #$00
   JSR LAB_LIST
      
	
   lda #"S"
	jsr write
   lda #"A"
   jsr write
   lda #"V"
   jsr write
   lda #"E"
   jsr write
   lda #$0d  
   jsr write
      
   
   jsr writeenable
   lda #CS2
	sta PORTB
	lda #DATA_WRITE
	jsr spi_transceive
	
   lda #00
   jsr spi_transceive ;adress 2
   lda #00
	jsr spi_transceive ;adress 1     
	
   lda ADLO
   jsr spi_transceive ;byte
   
   lda ADHI
   jsr spi_transceive ;byte
   
   lda #CSH
	sta PORTB
	
	LDA #$0
   sta SFLAG
   
   jsr DELAY
   LDA #"D"
   jsr write
   
   jsr DELAY 
   
   LDA #"E"
   jsr write
      
  lda #$00
  sta ADSAVELO
  lda #$50
  sta ADSAVEHI
  
   jsr writeenable
   lda #CS2
	sta PORTB
	lda #DATA_WRITE
	jsr spi_transceive
	
	lda #00
   jsr spi_transceive ;adress 2
   lda #02
	jsr spi_transceive ;adress 1     
	LDA #"S"
   jsr write
   

	ldy #$00
	ldx #$30
	dex
	dex
	dex
onemore: 
	lda (ADSAVELO),Y
	jsr write 
	jsr spi_transceive ;adress 1     
	iny
   dex	
   bne onemore
   
   LDA #"X"
   jsr write 
   lda #CSH
	sta PORTB
	jsr DELAY
	jsr DELAY
	LDA #"O"
	jsr write 
	LDA #"K"
	jsr write
   jsr DELAY
	jsr DELAY
	
	LDA #$0
   sta SFLAG	
	RTS

; vector tables

LAB_vec
	.word	ACIAin		; byte in 
	.word	ACIAout		; byte out
	.word	no_load		   ; load vector for EhBASIC
	.word	no_save		   ; save vector for EhBASIC

; EhBASIC IRQ support

IRQ_CODE
	;PHA				; save A
	;LDA	IrqBase		; get the IRQ flag byte
	;LSR				; shift the set b7 to b6, and on down ...
	;ORA	IrqBase		; OR the original back in
	;STA	IrqBase		; save the new IRQ flag byte
	;PLA				; restore A
	RTI

; EhBASIC NMI support

NMI_CODE
	;PHA				; save A
	;LDA	NmiBase		; get the NMI flag byte
	;LSR				; shift the set b7 to b6, and on down ...
	;ORA	NmiBase		; OR the original back in
	;STA	NmiBase		; save the new NMI flag byte
	;PLA				; restore A
	RTI

END_CODE

LAB_mess:
	.byte	   " EK 6502 SBC Bios.   2021-2022.",$0d,$0a
	;        1   2   3   4   5   6   7   8   9   0   1   2   3   4   5   6   7   8   9   0   1   2   3   4   5   6   7   8   9   0   1   2   3   4   5   6   7   8   9   0   1   2   3   4   5   6   7   8   9
	.byte	$c9,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$d1,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$bb,$0d,$0a
	;        1 234567890123456789012345678901234567890123456789
	.byte	$ba," CPU: W65C02      ",$b3,"  Timer:$0400  SR:$A01a ",$ba,$0d,$0a
	.byte	$ba," RAM: 0-7fff      ",$b3,"  Port1:$A010           ",$ba,$0d,$0a,$00
LAB_mess2:
	.byte	$ba," vga: 80000-87FF  ",$b3,"  Port2:$A020           ",$ba,$0d,$0a
	.byte	$c8,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cf,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$cd,$bc,$0d,$0a,$00
	

	*=	$FC00

keymap:
  .byte "????????????? `?"		   ; 00-0F
  .byte "?????q1???zsaw2?" 			; 10-1F
  .byte "?cxde43?? vftr5?" 			; 20-2F
  .byte "?nbhgy6???mju78?" 			; 30-3F
  .byte "?,kio09??./l;p-?" 			; 40-4F
  .byte "??'?[=????",$0d,"]?\??" 	; 50-5F
  .byte "??????",$08, "??1?47???" ; 60-6F
  .byte "0.2568",$1b,"??+3-*9",$03,"?"  ; 70-7F
  .byte "????????????????" 	      ; 80-8F
  .byte "????????????????" 			; 90-9F
  .byte "????????????????" 			; A0-AF
  .byte "????????????????" 			; B0-BF
  .byte "????????????????" 			; C0-CF
  .byte "????????????????" 			; D0-DF
  .byte "????????????????" 			; E0-EF
  .byte "????????????????" 			; F0-FF
keymap_shifted:
  .byte "????????????? ~?" 			; 00-0F
  .byte "?????Q!???ZSAW@?" 			; 10-1F
  .byte "?CXDE$#?? VFTR%?" 			; 20-2F
  .byte "?NBHGY^???MJU&*?" 			; 30-3F
  .byte "?<KIO)(??>?L:P_?" 			; 40-4F
  .byte "??",$22,"?{+????",$0d,"}?|??" 		; 50-5F
  .byte "??????",$08, "??1?47???" 			; 60-6F
  .byte "0.2568???+3-*9",$03,"?" 			; 70-7F
  .byte "????????????????" 			; 80-8F
  .byte "????????????????" 			; 90-9F
  .byte "????????????????" 			; A0-AF
  .byte "????????????????" 			; B0-BF
  .byte "????????????????" 			; C0-CF
  .byte "????????????????" 			; D0-DF
  .byte "????????????????" 			; E0-EF
  .byte "????????????????" 			; F0-F
keymap_caps:
  .byte "????????????? `?"		   ; 00-0F
  .byte "?????q1???ZSAW2?" 			; 10-1F
  .byte "?CXDE43?? VFTR5?" 			; 20-2F
  .byte "?NBHGY6???MJU78?" 			; 30-3F
  .byte "?,KIO09??./L;P-?" 			; 40-4F
  .byte "??'?[=????",$0d,"]?\??" 	; 50-5F
  .byte "??????",$08, "??1?47???" ; 60-6F
  .byte "0.2568",$1b,"??+3-*9",$03,"?"  ; 70-7F
  .byte "????????????????" 	      ; 80-8F
  .byte "????????????????" 			; 90-9F
  .byte "????????????????" 			; A0-AF
  .byte "????????????????" 			; B0-BF
  .byte "????????????????" 			; C0-CF
  .byte "????????????????" 			; D0-DF
  .byte "????????????????" 			; E0-EF
  .byte "????????????????" 			; F0-FF



; system vectors


nmi:
  rti
  
	*=	$FFFA

	.word	nmi		; NMI vector
	.word	RES_vec		; RESET vector
   .word	irq		; IRQ vector


;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------

; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz
;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------
	.def getMSP
	.def getPSP
	.def getR1
	.def getR0
	.def getR2
	.def getR3
	.def getR12
	.def getLR
	.def getPC
	.def getxPSR
	.def setPSP
	.def setMSP
	.def changeSPtoPSP
	.def change_Privilege	;This changes from unprivileged to privileged execution only in Handler Mode
;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb


;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------
.text

;Function that gets the value of stack pointer
getMSP:
		MRS R0,MSP
		BX LR

getPSP:
		MRS R0,PSP
		BX LR

setPSP:
		MSR PSP,R0
		BX LR

setMSP:
		MSR MSP,R0
		BX LR

changeSPtoPSP:
		MOV R0,#0x02
		MSR CONTROL,R0
		BX LR

change_Privilege:
		MRS R0,CONTROL
		EOR R0,R0,#0x01
		MSR CONTROL,R0
		BX LR

getR0:
		MRS R0,PSP
		LDR R0,[R0]
		BX LR

getR1:
		MRS R0,PSP
		ADD R0,#4
		LDR R0,[R0]
		BX LR

getR2:
		MRS R0,PSP
		ADD R0,#8
		LDR R0,[R0]
		BX LR

getR3:
		MRS R0,PSP
		ADD R0,#12
		LDR R0,[R0]
		BX LR

getR12:
		MRS R0,PSP
		ADD R0,#16
		LDR R0,[R0]
		BX LR

getLR:
		MRS R0,PSP
		ADD R0,#20
		LDR R0,[R0]
		BX LR

getPC:
		MRS R0,PSP
		ADD R0,#24
		LDR R0,[R0]
		BX LR

getxPSR:
		MRS R0,PSP
		ADD R0,#28
		LDR R0,[R0]
		BX LR
.endm

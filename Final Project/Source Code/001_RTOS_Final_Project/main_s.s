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
	.def setPSP
	.def setMSP
	.def changeSPtoPSP
	.def change_Privilege
	.def getSVCNumber
;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb


;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------
.text

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
		BX  LR

change_Privilege:
		MRS R0,CONTROL
		EOR R0,R0,#0x01
		MSR CONTROL,R0
		BX LR

getSVCNumber:
		MRS R0,PSP
		ADD R0,R0,#24
		LDR R0,[R0]
		SUB R0,R0,#2
		LDR R0,[R0]
		BX  LR
.endm

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------


	.def setPSP
	.def setASP
	.def getPSP
	.def pushReglist
	.def popReglist
	.def createHWpushContext
	.def getSVCnumber
	.def getR0

;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------


.thumb


.text

setASP:
				MRS  R1, CONTROL
				ORR  R1, R0
				MSR CONTROL, R1
				BX LR

setPSP:
				MSR PSP, R0
				BX LR

getPSP:
				MRS R0, PSP
				BX LR

pushReglist:
				MRS R0, PSP
				SUB R0, #4
				STR R4, [R0]
				SUB R0, #4
				STR R5, [R0]
				SUB R0, #4
				STR R6, [R0]
				SUB R0, #4
				STR R7, [R0]
				SUB R0, #4
				STR R8, [R0]
				SUB R0, #4
				STR R9, [R0]
				SUB R0, #4
				STR R10,[R0]
				SUB R0, #4
				STR R11,[R0]
				MSR PSP, R0
				BX LR

popReglist:
				MRS R0, PSP
				LDR R11, [R0]
				ADD R0, #4
				LDR R10, [R0]
				ADD R0, #4
				LDR R9, [R0]
				ADD R0, #4
				LDR R8, [R0]
				ADD R0, #4
				LDR R7, [R0]
				ADD R0, #4
				LDR R6, [R0]
				ADD R0, #4
				LDR R5, [R0]
				ADD R0, #4
				LDR R4, [R0]
				ADD R0, #4
				MSR PSP, R0
				BX LR

createHWpushContext:
			   MRS R4, PSP
			   SUB R4, #4
			   STR R0, [R4] ; XPSR
			   SUB R4, #4
			   STR R1, [R4]  ; PC
			   SUB R4, #4
			   STR R2, [R4]  ; other registers - LR
			   SUB R4, #4
			   ADD R2, #1
			   STR R2, [R4]  ; R12
			   SUB R4, #4
			   ADD R2, #1
			   STR R2, [R4]  ; R3
			   SUB R4, #4
			   ADD R2, #1
			   STR R2, [R4]  ; R2
			   SUB R4, #4
			   ADD R2, #1
			   STR R2, [R4]	 ; R1
			   SUB R4, #4
			   ADD R2, #1
			   STR R2, [R4]	 ; R0
			   MSR PSP, R4   ; PSP pointed back
			   BX LR

getSVCnumber:
			   MRS R0, PSP
			   ADD R0, #24
			   LDR R0, [R0]
			   SUB R0, #2
			   BX LR

getR0:
			   BX LR
.endm

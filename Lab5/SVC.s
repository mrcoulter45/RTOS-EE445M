;/*****************************************************************************/
;/* OSasm.s: low-level OS commands, written in assembly                       */
;/*****************************************************************************/
;Jonathan Valvano/Andreas Gerstlauer, OS Lab 5 solution, 2/28/16


        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

		IMPORT	OS_Id            ;EXPORT (or GLOBAL) -> declares a symbol that can be used by the linker to resolve symbol references in separate object and library files
        IMPORT  OS_Sleep         ;EXTERN -> provide the assembler with a name that is not defined in the current assembly. imports the symbol only if it is referred to in the current assembly.
		IMPORT	OS_Kill
		IMPORT	OS_Time
		IMPORT	OS_AddThread
		EXPORT  SVC_Handler	


SVC_Handler
 	LDR R12,[SP,#24] ; Return address
	LDRH R12,[R12,#-2] ; SVC instruction is 2 bytes
	BIC R12,#0xFF00 ; Extract ID in R12
	LDM SP,{R0-R3} ; Get any parameters
	
	CMP R12, #0 ;CMP -> R12-op2
	BEQ id
	CMP R12, #1
	BEQ kill 
	CMP R12, #2
	BEQ sleep 
	CMP R12, #3
	BEQ time 
	CMP R12, #4
	BEQ addT
	
id PUSH{LR}
	  BL OS_Id
	  POP{LR}
      B done
      
sleep PUSH{LR}
      BL OS_Sleep
      POP{LR}
      B done
      
kill PUSH{LR}
      BL OS_Kill
      POP{LR}
      B done
      
time PUSH{LR}
      BL OS_Time
      POP{LR}
      B done
      
addT PUSH{LR}
      BL OS_AddThread
      POP{LR}
      B done

done
	STR R0,[SP] ; Store return value
	BX LR ; Return from exception

    ALIGN
    END

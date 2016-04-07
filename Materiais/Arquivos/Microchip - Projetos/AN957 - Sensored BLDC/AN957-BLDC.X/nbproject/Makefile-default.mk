#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED="../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Init.c" "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Interrupts.c" "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/SensoredBLDC.c"

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1808442485/Init.o ${OBJECTDIR}/_ext/1808442485/Interrupts.o ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1808442485/Init.o.d ${OBJECTDIR}/_ext/1808442485/Interrupts.o.d ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1808442485/Init.o ${OBJECTDIR}/_ext/1808442485/Interrupts.o ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o

# Source Files
SOURCEFILES=../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Init.c ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Interrupts.c ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/SensoredBLDC.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ32MC204
MP_LINKER_FILE_OPTION=,--script=../../../../../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/gld/p33FJ32MC204.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1808442485/Init.o: ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto\ Sensored\ BLDC/Init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1808442485" 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Init.c"  -o ${OBJECTDIR}/_ext/1808442485/Init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1808442485/Init.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=coff -no-legacy-libc  -O0 -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h" -I"../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC" -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h/peripheral_30F_24H_33F" -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1808442485/Init.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1808442485/Interrupts.o: ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto\ Sensored\ BLDC/Interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1808442485" 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Interrupts.c"  -o ${OBJECTDIR}/_ext/1808442485/Interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1808442485/Interrupts.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=coff -no-legacy-libc  -O0 -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h" -I"../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC" -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h/peripheral_30F_24H_33F" -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1808442485/Interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o: ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto\ Sensored\ BLDC/SensoredBLDC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1808442485" 
	@${RM} ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/SensoredBLDC.c"  -o ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=coff -no-legacy-libc  -O0 -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h" -I"../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC" -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h/peripheral_30F_24H_33F" -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1808442485/Init.o: ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto\ Sensored\ BLDC/Init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1808442485" 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Init.c"  -o ${OBJECTDIR}/_ext/1808442485/Init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1808442485/Init.o.d"        -g -omf=coff -no-legacy-libc  -O0 -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h" -I"../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC" -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h/peripheral_30F_24H_33F" -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1808442485/Init.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1808442485/Interrupts.o: ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto\ Sensored\ BLDC/Interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1808442485" 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1808442485/Interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/Interrupts.c"  -o ${OBJECTDIR}/_ext/1808442485/Interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1808442485/Interrupts.o.d"        -g -omf=coff -no-legacy-libc  -O0 -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h" -I"../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC" -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h/peripheral_30F_24H_33F" -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1808442485/Interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o: ../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto\ Sensored\ BLDC/SensoredBLDC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1808442485" 
	@${RM} ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC/SensoredBLDC.c"  -o ${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d"        -g -omf=coff -no-legacy-libc  -O0 -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h" -I"../../../../Codigos/Microchip/PIC33FJ32MC204/Projeto Sensored BLDC" -I"../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/h/peripheral_30F_24H_33F" -I"." -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1808442485/SensoredBLDC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../../../../../Program\ Files/Microchip/MPLAB\ C30/lib/libp33FJ32MC204-coff.a  ../../../../../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/gld/p33FJ32MC204.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    "..\..\..\..\..\..\..\..\..\..\Program Files\Microchip\MPLAB C30\lib\libp33FJ32MC204-coff.a"  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=coff -no-legacy-libc   -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/gld",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/AN957-BLDC.X.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../../../../../Program\ Files/Microchip/MPLAB\ C30/lib/libp33FJ32MC204-coff.a ../../../../../../../../../../Program\ Files/Microchip/MPLAB\ C30/support/gld/p33FJ32MC204.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    "..\..\..\..\..\..\..\..\..\..\Program Files\Microchip\MPLAB C30\lib\libp33FJ32MC204-coff.a"  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=coff -no-legacy-libc  -Wl,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="../../../../../../../../../../Program Files/Microchip/MPLAB C30/support/gld",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/AN957-BLDC.X.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/AN957-BLDC.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=coff  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif

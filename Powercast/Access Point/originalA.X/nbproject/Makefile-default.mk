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
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../HardwareProfile.c ../AccessPoint.c ../../../Microchip/Common/Console.c ../../../Microchip/Common/MSPI.c ../../../Microchip/Common/TimeDelay.c ../../../Microchip/WirelessProtocols/MiWi/MiWi.c ../../../Microchip/WirelessProtocols/P2P/P2P.c ../../../Microchip/WirelessProtocols/LCDBlocking.c ../../../Microchip/WirelessProtocols/NVM.c ../../../Microchip/WirelessProtocols/SymbolTime.c ../../../Microchip/Transceivers/crc.c ../../../Microchip/Transceivers/security.c ../../../Microchip/Transceivers/MRF24J40/MRF24J40.c ../../../Microchip/Transceivers/MRF49XA/MRF49XA.c ../../../Microchip/Transceivers/MRF89XA/MRF89XA.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/HardwareProfile.o ${OBJECTDIR}/_ext/1472/AccessPoint.o ${OBJECTDIR}/_ext/1860783399/Console.o ${OBJECTDIR}/_ext/1860783399/MSPI.o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o ${OBJECTDIR}/_ext/35256690/MiWi.o ${OBJECTDIR}/_ext/1109514802/P2P.o ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o ${OBJECTDIR}/_ext/1466186449/NVM.o ${OBJECTDIR}/_ext/1466186449/SymbolTime.o ${OBJECTDIR}/_ext/1127682237/crc.o ${OBJECTDIR}/_ext/1127682237/security.o ${OBJECTDIR}/_ext/1061806389/MRF24J40.o ${OBJECTDIR}/_ext/1696884637/MRF49XA.o ${OBJECTDIR}/_ext/1697003801/MRF89XA.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/HardwareProfile.o.d ${OBJECTDIR}/_ext/1472/AccessPoint.o.d ${OBJECTDIR}/_ext/1860783399/Console.o.d ${OBJECTDIR}/_ext/1860783399/MSPI.o.d ${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d ${OBJECTDIR}/_ext/35256690/MiWi.o.d ${OBJECTDIR}/_ext/1109514802/P2P.o.d ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d ${OBJECTDIR}/_ext/1466186449/NVM.o.d ${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d ${OBJECTDIR}/_ext/1127682237/crc.o.d ${OBJECTDIR}/_ext/1127682237/security.o.d ${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d ${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d ${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/HardwareProfile.o ${OBJECTDIR}/_ext/1472/AccessPoint.o ${OBJECTDIR}/_ext/1860783399/Console.o ${OBJECTDIR}/_ext/1860783399/MSPI.o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o ${OBJECTDIR}/_ext/35256690/MiWi.o ${OBJECTDIR}/_ext/1109514802/P2P.o ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o ${OBJECTDIR}/_ext/1466186449/NVM.o ${OBJECTDIR}/_ext/1466186449/SymbolTime.o ${OBJECTDIR}/_ext/1127682237/crc.o ${OBJECTDIR}/_ext/1127682237/security.o ${OBJECTDIR}/_ext/1061806389/MRF24J40.o ${OBJECTDIR}/_ext/1696884637/MRF49XA.o ${OBJECTDIR}/_ext/1697003801/MRF89XA.o

# Source Files
SOURCEFILES=../HardwareProfile.c ../AccessPoint.c ../../../Microchip/Common/Console.c ../../../Microchip/Common/MSPI.c ../../../Microchip/Common/TimeDelay.c ../../../Microchip/WirelessProtocols/MiWi/MiWi.c ../../../Microchip/WirelessProtocols/P2P/P2P.c ../../../Microchip/WirelessProtocols/LCDBlocking.c ../../../Microchip/WirelessProtocols/NVM.c ../../../Microchip/WirelessProtocols/SymbolTime.c ../../../Microchip/Transceivers/crc.c ../../../Microchip/Transceivers/security.c ../../../Microchip/Transceivers/MRF24J40/MRF24J40.c ../../../Microchip/Transceivers/MRF49XA/MRF49XA.c ../../../Microchip/Transceivers/MRF89XA/MRF89XA.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=24F16KA102
MP_LINKER_FILE_OPTION=,--script=p24F16KA102.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/HardwareProfile.o: ../HardwareProfile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/HardwareProfile.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/HardwareProfile.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../HardwareProfile.c  -o ${OBJECTDIR}/_ext/1472/HardwareProfile.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/HardwareProfile.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/HardwareProfile.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/AccessPoint.o: ../AccessPoint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/AccessPoint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/AccessPoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../AccessPoint.c  -o ${OBJECTDIR}/_ext/1472/AccessPoint.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/AccessPoint.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/AccessPoint.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/Console.o: ../../../Microchip/Common/Console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/Console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/Console.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/Console.c  -o ${OBJECTDIR}/_ext/1860783399/Console.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/Console.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/Console.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/MSPI.o: ../../../Microchip/Common/MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/MSPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/MSPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/MSPI.c  -o ${OBJECTDIR}/_ext/1860783399/MSPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/MSPI.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/MSPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/TimeDelay.o: ../../../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/TimeDelay.c  -o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/35256690/MiWi.o: ../../../Microchip/WirelessProtocols/MiWi/MiWi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/35256690" 
	@${RM} ${OBJECTDIR}/_ext/35256690/MiWi.o.d 
	@${RM} ${OBJECTDIR}/_ext/35256690/MiWi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/MiWi/MiWi.c  -o ${OBJECTDIR}/_ext/35256690/MiWi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/35256690/MiWi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/35256690/MiWi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1109514802/P2P.o: ../../../Microchip/WirelessProtocols/P2P/P2P.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1109514802" 
	@${RM} ${OBJECTDIR}/_ext/1109514802/P2P.o.d 
	@${RM} ${OBJECTDIR}/_ext/1109514802/P2P.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/P2P/P2P.c  -o ${OBJECTDIR}/_ext/1109514802/P2P.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1109514802/P2P.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1109514802/P2P.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1466186449/LCDBlocking.o: ../../../Microchip/WirelessProtocols/LCDBlocking.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1466186449" 
	@${RM} ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d 
	@${RM} ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/LCDBlocking.c  -o ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1466186449/NVM.o: ../../../Microchip/WirelessProtocols/NVM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1466186449" 
	@${RM} ${OBJECTDIR}/_ext/1466186449/NVM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1466186449/NVM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/NVM.c  -o ${OBJECTDIR}/_ext/1466186449/NVM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1466186449/NVM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1466186449/NVM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1466186449/SymbolTime.o: ../../../Microchip/WirelessProtocols/SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1466186449" 
	@${RM} ${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d 
	@${RM} ${OBJECTDIR}/_ext/1466186449/SymbolTime.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/SymbolTime.c  -o ${OBJECTDIR}/_ext/1466186449/SymbolTime.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1127682237/crc.o: ../../../Microchip/Transceivers/crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1127682237" 
	@${RM} ${OBJECTDIR}/_ext/1127682237/crc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1127682237/crc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/crc.c  -o ${OBJECTDIR}/_ext/1127682237/crc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1127682237/crc.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1127682237/crc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1127682237/security.o: ../../../Microchip/Transceivers/security.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1127682237" 
	@${RM} ${OBJECTDIR}/_ext/1127682237/security.o.d 
	@${RM} ${OBJECTDIR}/_ext/1127682237/security.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/security.c  -o ${OBJECTDIR}/_ext/1127682237/security.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1127682237/security.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1127682237/security.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1061806389/MRF24J40.o: ../../../Microchip/Transceivers/MRF24J40/MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1061806389" 
	@${RM} ${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1061806389/MRF24J40.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/MRF24J40/MRF24J40.c  -o ${OBJECTDIR}/_ext/1061806389/MRF24J40.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1696884637/MRF49XA.o: ../../../Microchip/Transceivers/MRF49XA/MRF49XA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1696884637" 
	@${RM} ${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d 
	@${RM} ${OBJECTDIR}/_ext/1696884637/MRF49XA.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/MRF49XA/MRF49XA.c  -o ${OBJECTDIR}/_ext/1696884637/MRF49XA.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1697003801/MRF89XA.o: ../../../Microchip/Transceivers/MRF89XA/MRF89XA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1697003801" 
	@${RM} ${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d 
	@${RM} ${OBJECTDIR}/_ext/1697003801/MRF89XA.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/MRF89XA/MRF89XA.c  -o ${OBJECTDIR}/_ext/1697003801/MRF89XA.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1    -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1472/HardwareProfile.o: ../HardwareProfile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/HardwareProfile.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/HardwareProfile.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../HardwareProfile.c  -o ${OBJECTDIR}/_ext/1472/HardwareProfile.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/HardwareProfile.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/HardwareProfile.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/AccessPoint.o: ../AccessPoint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/AccessPoint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/AccessPoint.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../AccessPoint.c  -o ${OBJECTDIR}/_ext/1472/AccessPoint.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/AccessPoint.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/AccessPoint.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/Console.o: ../../../Microchip/Common/Console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/Console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/Console.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/Console.c  -o ${OBJECTDIR}/_ext/1860783399/Console.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/Console.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/Console.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/MSPI.o: ../../../Microchip/Common/MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/MSPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/MSPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/MSPI.c  -o ${OBJECTDIR}/_ext/1860783399/MSPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/MSPI.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/MSPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1860783399/TimeDelay.o: ../../../Microchip/Common/TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1860783399" 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1860783399/TimeDelay.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Common/TimeDelay.c  -o ${OBJECTDIR}/_ext/1860783399/TimeDelay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1860783399/TimeDelay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/35256690/MiWi.o: ../../../Microchip/WirelessProtocols/MiWi/MiWi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/35256690" 
	@${RM} ${OBJECTDIR}/_ext/35256690/MiWi.o.d 
	@${RM} ${OBJECTDIR}/_ext/35256690/MiWi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/MiWi/MiWi.c  -o ${OBJECTDIR}/_ext/35256690/MiWi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/35256690/MiWi.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/35256690/MiWi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1109514802/P2P.o: ../../../Microchip/WirelessProtocols/P2P/P2P.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1109514802" 
	@${RM} ${OBJECTDIR}/_ext/1109514802/P2P.o.d 
	@${RM} ${OBJECTDIR}/_ext/1109514802/P2P.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/P2P/P2P.c  -o ${OBJECTDIR}/_ext/1109514802/P2P.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1109514802/P2P.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1109514802/P2P.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1466186449/LCDBlocking.o: ../../../Microchip/WirelessProtocols/LCDBlocking.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1466186449" 
	@${RM} ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d 
	@${RM} ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/LCDBlocking.c  -o ${OBJECTDIR}/_ext/1466186449/LCDBlocking.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1466186449/LCDBlocking.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1466186449/NVM.o: ../../../Microchip/WirelessProtocols/NVM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1466186449" 
	@${RM} ${OBJECTDIR}/_ext/1466186449/NVM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1466186449/NVM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/NVM.c  -o ${OBJECTDIR}/_ext/1466186449/NVM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1466186449/NVM.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1466186449/NVM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1466186449/SymbolTime.o: ../../../Microchip/WirelessProtocols/SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1466186449" 
	@${RM} ${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d 
	@${RM} ${OBJECTDIR}/_ext/1466186449/SymbolTime.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/WirelessProtocols/SymbolTime.c  -o ${OBJECTDIR}/_ext/1466186449/SymbolTime.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1466186449/SymbolTime.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1127682237/crc.o: ../../../Microchip/Transceivers/crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1127682237" 
	@${RM} ${OBJECTDIR}/_ext/1127682237/crc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1127682237/crc.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/crc.c  -o ${OBJECTDIR}/_ext/1127682237/crc.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1127682237/crc.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1127682237/crc.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1127682237/security.o: ../../../Microchip/Transceivers/security.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1127682237" 
	@${RM} ${OBJECTDIR}/_ext/1127682237/security.o.d 
	@${RM} ${OBJECTDIR}/_ext/1127682237/security.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/security.c  -o ${OBJECTDIR}/_ext/1127682237/security.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1127682237/security.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1127682237/security.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1061806389/MRF24J40.o: ../../../Microchip/Transceivers/MRF24J40/MRF24J40.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1061806389" 
	@${RM} ${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d 
	@${RM} ${OBJECTDIR}/_ext/1061806389/MRF24J40.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/MRF24J40/MRF24J40.c  -o ${OBJECTDIR}/_ext/1061806389/MRF24J40.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1061806389/MRF24J40.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1696884637/MRF49XA.o: ../../../Microchip/Transceivers/MRF49XA/MRF49XA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1696884637" 
	@${RM} ${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d 
	@${RM} ${OBJECTDIR}/_ext/1696884637/MRF49XA.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/MRF49XA/MRF49XA.c  -o ${OBJECTDIR}/_ext/1696884637/MRF49XA.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1696884637/MRF49XA.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1697003801/MRF89XA.o: ../../../Microchip/Transceivers/MRF89XA/MRF89XA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1697003801" 
	@${RM} ${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d 
	@${RM} ${OBJECTDIR}/_ext/1697003801/MRF89XA.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../../../Microchip/Transceivers/MRF89XA/MRF89XA.c  -o ${OBJECTDIR}/_ext/1697003801/MRF89XA.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d"        -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"../../../MiWi DE Demo/Node 2" -I"../../../Microchip/Include" -I"../../../Microchip/Include/Transceivers" -I"../../../Microchip/Include/Transceivers/MRF24J40" -I"../../../Microchip/Include/Transceivers/MRF49XA" -I"../../../Microchip/Include/Transceivers/MRF89XA" -I"../../../Microchip/Include/WirelessProtocols" -I"../../../Microchip/Include/WirelessProtocols/MiWi" -I"../../../Microchip/Include/WirelessProtocols/P2P" -I"." -DSIMPLE_EXAMPLE -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/_ext/1697003801/MRF89XA.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
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
dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.originalA.X.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.originalA.X.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/originalA.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
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

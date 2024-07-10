################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"C:/Users/office-win/Desktop/CCS/ccs/tools/compiler/ti-cgt-pru_2.3.3/bin/clpru" -Ooff --opt_for_speed=0 --include_path="C:/Users/office-win/Desktop/CCS/ccs/ccs_base/pru/include" --include_path="C:/Users/office-win/Desktop/CCS/projects/PruLibrary/src" --include_path="C:/Users/office-win/Desktop/CCS/projects/ti/pru-software-support-package-6.4.0/include" --include_path="C:/Users/office-win/Desktop/CCS/projects/ti/pru-software-support-package-6.4.0/include/j721e" --include_path="C:/Users/office-win/Desktop/CCS/projects/ti/pru-software-support-package-6.4.0/include/am64x" --include_path="C:/Users/office-win/Desktop/CCS/ccs/tools/compiler/ti-cgt-pru_2.3.3/include" --include_path="C:/Users/office-win/Desktop/CCS/ccs/ccs_base/pru/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



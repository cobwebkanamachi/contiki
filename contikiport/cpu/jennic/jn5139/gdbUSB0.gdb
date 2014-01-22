set ignore_MMU 1
set remotelogfile remoteUSB0.log
set remotebaud 38400
target remote /dev/ttyUSB0

#break vm.c:231
#disp /x pvm_instr
#disp pvm_pc
#disp sp - ((pvm_stack_t*) stack_space)
#disp /x repo_current_header.classuid
#disp pvm_arg0

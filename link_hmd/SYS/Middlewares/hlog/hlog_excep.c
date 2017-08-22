#include "hlog_inc.h"

#define is_text_address(pc)	1
#define STACK_EXTRA_CHECK_SIZE	(4*16)

struct pt_regs {
        unsigned long regs[18];
        unsigned long exc_return;
        unsigned long xmask;
        unsigned long cfsr;
        unsigned long hfsr;
};

#define XMASK_IPSR(xmask)             ((xmask) & 0x1ff)
#define XMASK_BASEPRI(xmask)       (((xmask) >> 16) & 0xff)
#define XMASK_PRIMASK(xmask)       (((xmask) >> 24) & 0x01)
#define XMASK_FAULTMASK(xmask)       (((xmask) >> 28) & 0x01)

static int hlog_expinfo_printf(char *fmt, ...)
{
        int retval;
        va_list vp;
        char logmsg_buf[CONFIG_LOGMSG_BUF_SIZE];

        va_start(vp, fmt);
        retval = vsprintf(logmsg_buf, fmt, vp);
        va_end(vp);

        watchdog_refresh();
        __write_logbuf_force(logmsg_buf, retval);
        dbg_uart_write(logmsg_buf, retval);

        return retval;
}

#undef 	hlog_exp_printf
#define   hlog_exp_printf	hlog_expinfo_printf

static void __show_regs(struct pt_regs *regs)
{
        int i;
        volatile unsigned long  *shpr1;

        regs->cfsr = SCB->CFSR;
        regs->hfsr = SCB->HFSR;

        for (i = 0; i < 16; i += 4) {
                hlog_exp_printf("$%2d   : %08lx %08lx %08lx %08lx\n", i,
                regs->regs[i], regs->regs[i+1], regs->regs[i+2], regs->regs[i+3]);
        }
        hlog_exp_printf("$%2d   : %08lx %08lx %08lx %08lx\n", i,
                regs->regs[16], regs->regs[17], regs->exc_return, regs->xmask);

        //hlog_exp_printf("EXCEP_TYPE = %s\n", "Watchdog Timeout");

        hlog_exp_printf("CFSR  = 0x%08x HFSR  = 0x%08x\n", regs->cfsr, regs->hfsr);
        if (regs->cfsr & 0x80)
                hlog_exp_printf("MMFAR = 0x%08x\n", SCB->MMFAR);
        if (regs->cfsr & 0x8000)
                hlog_exp_printf("BFAR  = 0x%08x\n", SCB->BFAR);
        hlog_exp_printf("ICSR  = 0x%08x SHCSR = 0x%08x\n", SCB->ICSR, SCB->SHCSR);

 #if 1
        hlog_exp_printf("ACTLR = 0x%08x AIRCR = 0x%08x\n", readl(0xe000e008), SCB->AIRCR);
        hlog_exp_printf("SCR   = 0x%08x CCR   = 0x%08x\n", SCB->SCR, SCB->CCR);
        shpr1 = (volatile unsigned long  *)SCB->SHP;
        hlog_exp_printf("SHPR1 = 0x%08x SHPR2 = 0x%08x SHPR3 = 0x%08x\n", shpr1[0], shpr1[1], shpr1[2]);
 #endif
        return ;
}

#if 0
static unsigned long unwind_stack_backward(unsigned long *sp, unsigned long pc, unsigned long *ra)
{
	int tmp;
	int frame_size;
	int ra_offset;
	uint16_t *func_addr;
	uint16_t *mm_pc;
	extern void ret_from_exception(void);

	if (*sp < (unsigned long)__stack_start__ || *sp > (unsigned long)__stack)
		return 0;

	if ((unsigned long)ret_from_exception == pc) {
		struct pt_regs *regs;
		regs = (struct pt_regs *)*sp;
		pc = regs->cp0_epc;
		if (is_text_address(pc)) {
			*sp = regs->regs[29];
			*ra = regs->regs[31];
			return pc;
		}
		return 0;
	}

	frame_size = 0;
	ra_offset = -1;
	mm_pc = (uint16_t *)(pc & ~1UL);
	func_addr = mm_pc;

	while (func_addr + BACKTRACE_MAX_INSTR_CHECK > mm_pc) {

		tmp = get_frame_size(func_addr);
		if (tmp) {
			if (func_addr_check(func_addr, mm_pc)) {
				if (tmp > 0)
					frame_size = tmp;
				break;
			}
		}

		tmp = get_ra_offset(func_addr);
		if (tmp >= 0)
			if (func_addr_check(func_addr, mm_pc))
				ra_offset = tmp;


		tmp = is_end_of_function_marker(func_addr);
		if (tmp)
			break;

		tmp = previous_instr_size(func_addr, 0);
		if (tmp < 0)
			break;
		func_addr = func_addr - tmp/sizeof(*func_addr);
	}
#if HLOG_BACKTRACE_DEBUG
	hlog_exp_printf("in %s : func_addr = 0x%08lx frame_size = %d, ra_offset = %d\n", __func__, (unsigned long)func_addr, frame_size, ra_offset);
#endif

	if (*sp + frame_size > (unsigned long)__stack)
		return 0;
#if 1
	/*
	* Return ra if an exception occurred at the first instruction
	*/
	if (func_addr == mm_pc) {
		pc = *ra;
		*ra = 0;
		return pc;
	}
#endif

	if (ra_offset < 0) {
		pc = pc != *ra ? *ra : 0;
	} else {
		pc = ((unsigned long *)*sp)[ra_offset];
	}

	*ra = 0;
	*sp += frame_size;
	return pc;
}
#endif

void show_backtrace(struct pt_regs *regs)
{
#if 0
	unsigned long sp = regs->regs[13];
	unsigned long ra = regs->regs[14];
#if 0
    unsigned long pc = regs->cp0_epc;
#else
    unsigned long pc = ra;
#endif

	if (NULL == regs) {
		__asm__ __volatile__ ("	move %0, $sp \n\t" : "=r"(sp));
		__asm__ __volatile__ (" move %0, $ra \n\t" : "=r"(ra));
		__asm__ __volatile__ ("1: la %0, 1b \n\t" : "=r"(pc));
	}

	if (!is_text_address(pc))
		return;

	hlog_exp_printf("\nCall Trace:\n\n");
	do {
		hlog_exp_printf("0x%08lx, 0x%08lx\n", pc, sp);
		pc = unwind_stack_backward(&sp, pc, &ra);
	} while (pc);

	hlog_exp_printf("\n");
#endif
}


static void show_stacktrace(struct pt_regs *regs)
{
        unsigned long tmp;
        unsigned long *sp;
        unsigned stack_start;
        unsigned stack_end;

        if ((regs->exc_return & 0x0c) == 0x0c) {
            vTask_get_current_taskinfo((unsigned long *)&stack_start, NULL);
            stack_end = vPort_get_malloc_size((void*)stack_start);
            if (0 == stack_end)
                    return;
            stack_end += stack_start;
        } else {
            stack_start =  __main_stack_start__;
            stack_end = __main_stack_end__;
        }

        tmp = regs->regs[13];
        if (tmp < stack_start || tmp >= stack_end)
                return ;

        hlog_exp_printf("Stack :\n");

        tmp -= STACK_EXTRA_CHECK_SIZE;
        tmp &= ~0x07;

        if (tmp < stack_start)
                tmp = stack_start;

        sp = (unsigned long*)tmp;

        while ((unsigned long)sp < (unsigned long)stack_end) {
            hlog_exp_printf("%08lx : %08lx %08lx %08lx %08lx  %08lx %08lx %08lx %08lx\n",
            (unsigned long)sp, sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], sp[7]);
            sp += 8;
        }

        show_backtrace(regs);
        return ;
}

static void show_code(void *pc)
{
	int i;
	unsigned short *pc16 = NULL;

	hlog_exp_printf("\nCode:\n");
	pc16 = (unsigned short *)((unsigned long)pc & ~1UL);

	for(i = -8 ; i < 16 ; i += 8)
		hlog_exp_printf("%08lx : %04x %04x %04x %04x  %04x %04x %04x %04x\n", (unsigned long)(pc16 + i),
						pc16[i], pc16[i+1], pc16[i+2], pc16[i+3], pc16[i+4], pc16[i+5], pc16[i+6], pc16[i+7]);
	return ;
}

static void show_registers(struct pt_regs *regs)
{
	show_stacktrace(regs);
	__show_regs(regs);
 #if 0
	show_code((unsigned int *) regs->cp0_epc);
 #else
	show_code((unsigned int *) regs->regs[15]);
 #endif
	return ;
}

static int hlog_grab_explog(void *args)
{
        int year, mon, day, hour, min, sec;
        char *cur_task_name;
        struct pt_regs *cpu_ctx = (struct pt_regs *)args;

        hlog_exp_printf("######## START EXP LOGS ########\r\n");

        get_calendar_time(&year, &mon, &day, &hour, &min, &sec);
        hlog_exp_printf("now : %04d-%02d-%02d %02d:%02d:%02d\r\n", year, mon, day, hour, min, sec);

        vTask_get_current_taskinfo(NULL, &cur_task_name);
        hlog_exp_printf("current process = %s\n", cur_task_name);
        /*
        * grab exception logs, TBD
        */
        show_registers(cpu_ctx);
        hlog_exp_printf("\r\n######## END EXP LOGS ########\r\n");

        //explog_flush();
        return 0;
}
extern char boot_type[16];

int hlog_exp_handler(void *args)
{
        //struct pt_regs *cpu_ctx = (struct pt_regs *)args;
        watchdog_refresh();

        hlog_grab_explog(args);
#if 1
        hlog_save_logbuf();
        hlog_flush_logbuf(0);
#endif
        memset(boot_type, 0, sizeof(boot_type));
        NVIC_SystemReset();
        while(1);
}


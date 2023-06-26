/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#if	IBM
#include <windows.h>
#include <dbghelp.h>
#include <psapi.h>
#else
#include <execinfo.h>   /* used for stack tracing */
#endif	/* !IBM */

#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/log.h>
#include <acfutils/thread.h>

#define	DATE_FMT	"%Y-%m-%d %H:%M:%S"
#define	PREFIX_FMT	"%s %s[%s:%d]: ", timedate, log_prefix, filename, line

#define	MAX_STACK_FRAMES	128
#define	MAX_MODULES		1024
#define	BACKTRACE_STR		"Backtrace is:\n"
#if	defined(__GNUC__) || defined(__clang__)
#define	BACKTRACE_STRLEN	__builtin_strlen(BACKTRACE_STR)
#else	/* !__GNUC__ && !__clang__ */
#define	BACKTRACE_STRLEN	strlen(BACKTRACE_STR)
#endif	/* !__GNUC__ && !__clang__ */

#if	IBM
/*
 * Since while dumping stack we are most likely in a fairly compromised
 * state, we statically pre-allocate these buffers to try and avoid having
 * to call into the VM subsystem.
 */
#define	MAX_SYM_NAME_LEN	1024
#define	MAX_BACKTRACE_LEN	(64 * 1024)
static char backtrace_buf[MAX_BACKTRACE_LEN] = { 0 };
static char symbol_buf[sizeof (SYMBOL_INFO) +
    MAX_SYM_NAME_LEN * sizeof (TCHAR)];
static char line_buf[sizeof (IMAGEHLP_LINE64)];
/* DbgHelp is not thread-safe, so avoid concurrency */
static mutex_t backtrace_lock;

static HMODULE modules[MAX_MODULES];
static MODULEINFO mi[MAX_MODULES];
static DWORD num_modules;

#define	SYMNAME_MAXLEN	1023	/* C++ symbols can be HUUUUGE */
#endif	/* IBM */

static logfunc_t log_func = NULL;
static char *log_prefix = NULL;

/**
 * Initializes the libacfutils logging subsystem. You must call this
 * before any other subsystem of libacfutils. Typically, you would do
 * this at the start of your plugin load callback (`XPluginStart()`).
 * This is to make sure that if any of libacfutils' subsystems or your
 * own code needs to log errors, they can. Without initialization, any
 * logging calls will cause the app to crash with a call to abort().
 *
 * At the end of your plugin's unload, deinitialize the logging
 * subsystem using log_fini().
 *
 * @param func A callback function which will be called for every log
 *	message that will be emitted. You can provide your own logging
 *	function, or use log_xplm_cb() to simply emit any log messages
 *	to the X-Plane Log.txt.
 * @param prefix A logging prefix name, which will be prepended to every
 *	log message. This is to help disambiguate which plugin is
 *	emitting a particular message. A good choice here is something
 *	that is a short name of your plugin, e.g. "my_plugin".
 */
void
log_init(logfunc_t func, const char *prefix)
{
	/* Can't use VERIFY here, since it uses this logging interface. */
	if (func == NULL || prefix == NULL)
		abort();
	log_func = func;
	log_prefix = safe_strdup(prefix);
#if	IBM
	mutex_init(&backtrace_lock);
#endif
}

/**
 * Deinitializes the logging system. You must call this at the end of your
 * plugin's unload function (`XPluginStop()`) to properly free any memory
 * resources used by the logging system.
 */
void
log_fini(void)
{
	free(log_prefix);
	log_prefix = NULL;
#if	IBM
	mutex_destroy(&backtrace_lock);
#endif
}

/**
 * A simple logging callback function suitable for passing to log_init()
 * in its first argument. This function simply emits the input string
 * to the X-Plane Log.txt file via XPLMDebugString().
 */
void
log_xplm_cb(const char *str)
{
	XPLMDebugString(str);
}

/**
 * Log implementation function. Do not call directly. Use the logMsg() macro.
 * @see logMsg()
 */
void
log_impl(const char *filename, int line, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	log_impl_v(filename, line, fmt, ap);
	va_end(ap);
}

/**
 * Log implementation function. Do not call directly. Use the logMsg_v() macro.
 * @see logMsg_v()
 */
void
log_impl_v(const char *filename, int line, const char *fmt, va_list ap)
{
	va_list ap_copy;
	char timedate[32];
	char *buf;
	size_t prefix_len, len;
	struct tm *tm;
	time_t t;

	t = time(NULL);
	tm = localtime(&t);
	VERIFY(strftime(timedate, sizeof (timedate), DATE_FMT, tm) != 0);

	/* Can't use VERIFY here, since it uses this logging interface. */
	if (log_func == NULL || log_prefix == NULL)
		abort();

	prefix_len = snprintf(NULL, 0, PREFIX_FMT);
	va_copy(ap_copy, ap);
	len = vsnprintf(NULL, 0, fmt, ap_copy);

	buf = (char *)malloc(prefix_len + len + 2);

	(void) snprintf(buf, prefix_len + 1, PREFIX_FMT);
	(void) vsnprintf(&buf[prefix_len], len + 1, fmt, ap);
	(void) sprintf(&buf[strlen(buf)], "\n");
	log_func(buf);

	free(buf);
}

/**
 * \func void log_backtrace(int skip_frames)
 * Logs a backtrace of the current stack to the logging system. This is
 * typically called from an assertion failure handler to try and gather
 * debugging information, before the app is terminated.
 *
 * This function uses OS-specific facilities to try and determine the
 * function names and offsets of each stack frame.
 *
 * - On macOS and Linux, this facility utilizes the `backtrace()` and
 *	`backtrace_symbols()` functions from the platforms' respective
 *	C libraries. These attempt to use any embedded symbol information
 *	inside of the binary to determine function names and offsets.
 * - On Windows, binaries unfortunately almost never ship with symbol
 *	information for internal symbols not exposed via `dllexport`.
 *	Therefore, to support getting *some* semblance of useful
 *	information, libacfutils utilizes a custom symbol table provided
 *	by you. This symbol table can be generated from a compiled binary
 *	containing DWARF2 debug information using the `mksyms` shell
 *	script contained in the `tools` directory of the libacfutils
 *	source repo. Please note, that MSVC doesn't generate DWARF2
 *	debug information and therefore this utility cannot be used for
 *	binaries compiled with MSVC. The generated file data should be
 *	redirected to a file named `syms.txt` and placed next to the
 *	Windows XPL file for the plugin and shipped with the plugin,
 *	like this:
 *```
 * $ tools/mksyms win.xpl > syms.txt
 *```
 *
 * @param skip_frames Number of stack frames to skip off the top of the
 *	stack before printing the remaining frames. Sometimes a crash
 *	handler is invoked from within an exception or signal handling
 *	callback. This would place a bunch of useless stack frames near
 *	the top of the backtrace, resulting in a backtrace that's hard
 *	to read. Use this argument to skip them, if you know for sure
 *	how many there are.
 */

#if	IBM

/*
 * Given a module path in `filename' and a relative module address in `addr',
 * attempts to resolve the symbol name and relative symbol address. This is
 * done by looking for a syms.txt file in the same directory as the module's
 * filename.
 * If found, the symbol name + symbol relative address is placed into
 * `symname' in the "symbol+offset" format.
 * This function is deliberately designed to be simple and use as little
 * memory as possible, because when called from an exception handler, the
 * process' memory state can be assumed to be quite broken already.
 */
static void
find_symbol(const char *filename, void *addr, char *symname,
    size_t symname_cap)
{
	/*
	 * Note the `static' here is deliberate to cause these to become
	 * BSS-allocated variables instead of stack-allocated. When parsing
	 * through a stack trace we are in a pretty precarious state, so we
	 * can't rely on having much stack space available.
	 */
	static char symstxtname[MAX_PATH];
	static char prevsym[SYMNAME_MAXLEN + 1];
	static const char *sep;
	static FILE *fp;
	static void *prevptr = NULL;
	static void *image_base = NULL;

	*symname = 0;
	*prevsym = 0;

	sep = strrchr(filename, DIRSEP);
	if (sep == NULL)
		return;
	lacf_strlcpy(symstxtname, filename, MIN((uintptr_t)(sep - filename) + 1,
	    sizeof (symstxtname)));
	strncat(symstxtname, DIRSEP_S "syms.txt", sizeof (symstxtname));
	fp = fopen(symstxtname, "rb");
	if (fp == NULL)
		return;

	while (!feof(fp)) {
		static char unused_c;
		static void *ptr;
		static char sym[SYMNAME_MAXLEN + 1];

		if (fscanf(fp, "%p %c %" SCANF_STR_AUTOLEN(SYMNAME_MAXLEN) "s",
		    &ptr, &unused_c, sym) != 3) {
			break;
		}
		if (strcmp(sym, "__image_base__") == 0) {
			image_base = ptr;
			continue;
		}
		ptr = (void *)(ptr - image_base);
		if (addr >= prevptr && addr < ptr) {
			snprintf(symname, symname_cap, "%s+%x", prevsym,
			    (unsigned)(addr - prevptr));
			break;
		}
		prevptr = ptr;
		lacf_strlcpy(prevsym, sym, sizeof (prevsym));
	}
	fclose(fp);
}

static HMODULE
find_module(LPVOID pc, DWORD64 *module_base)
{
	static DWORD i;
	for (i = 0; i < num_modules; i++) {
		static LPVOID start, end;
		start = mi[i].lpBaseOfDll;
		end = start + mi[i].SizeOfImage;
		if (start <= pc && end > pc) {
			*module_base = (DWORD64)start;
			return (modules[i]);
		}
	}
	*module_base = 0;
	return (NULL);
}

static void
gather_module_info(void)
{
	HANDLE process = GetCurrentProcess();

	EnumProcessModules(process, modules, sizeof (HMODULE) * MAX_MODULES,
	    &num_modules);
	num_modules = MIN(num_modules, MAX_MODULES);
	for (DWORD i = 0; i < num_modules; i++)
		GetModuleInformation(process, modules[i], &mi[i], sizeof (*mi));
}

void
log_backtrace(int skip_frames)
{
	static unsigned frames;
	static void *stack[MAX_STACK_FRAMES];
	static SYMBOL_INFO *symbol;
	static HANDLE process;
	static DWORD displacement;
	static IMAGEHLP_LINE64 *line;
	static char filename[MAX_PATH];

	frames = RtlCaptureStackBackTrace(skip_frames + 1, MAX_STACK_FRAMES,
	    stack, NULL);

	process = GetCurrentProcess();
	mutex_enter(&backtrace_lock);

	SymInitialize(process, NULL, TRUE);
	SymSetOptions(SYMOPT_LOAD_LINES);

	gather_module_info();

	memset(symbol_buf, 0, sizeof (symbol_buf));
	memset(line_buf, 0, sizeof (line_buf));

	symbol = (SYMBOL_INFO *)symbol_buf;
	symbol->MaxNameLen = MAX_SYM_NAME_LEN - 1;
	symbol->SizeOfStruct = sizeof (SYMBOL_INFO);

	line = (IMAGEHLP_LINE64 *)line_buf;
	line->SizeOfStruct = sizeof (*line);

	backtrace_buf[0] = '\0';
	lacf_strlcpy(backtrace_buf, BACKTRACE_STR, sizeof (backtrace_buf));

	for (unsigned frame_nr = 0; frame_nr < frames; frame_nr++) {
		static DWORD64 address;
		static int fill;

		address = (DWORD64)(uintptr_t)stack[frame_nr];
		fill = strlen(backtrace_buf);

		memset(symbol_buf, 0, sizeof (symbol_buf));
		/*
		 * Try to grab the symbol name from the stored %rip data.
		 */
		if (!SymFromAddr(process, address, 0, symbol)) {
			static DWORD64 start;
			static HMODULE module;

			module = find_module((void *)address, &start);
			if (module != NULL) {
				static char symname[SYMNAME_MAXLEN + 1];

				GetModuleFileNameA(module, filename,
				    sizeof (filename));
				find_symbol(filename, stack[frame_nr] - start,
				    symname, sizeof (symname));
				fill += snprintf(&backtrace_buf[fill],
				    sizeof (backtrace_buf) - fill,
				    "%d %p %s+%p (%s)\n", frame_nr,
				    stack[frame_nr], filename,
				    stack[frame_nr] - start, symname);
			} else {
				fill += snprintf(&backtrace_buf[fill],
				    sizeof (backtrace_buf) - fill,
				    "%d %p <unknown module>\n", frame_nr,
				    stack[frame_nr]);
			}
			continue;
		}
		/*
		 * See if we have debug info available with file names and
		 * line numbers.
		 */
		if (SymGetLineFromAddr64(process, address, &displacement,
		    line)) {
			snprintf(&backtrace_buf[fill], sizeof (backtrace_buf) -
			    fill, "%d: %s (0x%lx) [%s:%d]\n", frame_nr,
			    symbol->Name, (unsigned long)symbol->Address,
			    line->FileName, (int)line->LineNumber);
		} else {
			snprintf(&backtrace_buf[fill], sizeof (backtrace_buf) -
			    fill, "%d: %s - 0x%lx\n", frame_nr, symbol->Name,
			    (unsigned long)symbol->Address);
		}
	}

	if (log_func == NULL)
		abort();
	log_func(backtrace_buf);
	fputs(backtrace_buf, stderr);
	fflush(stderr);
	SymCleanup(process);

	mutex_exit(&backtrace_lock);
}

void
log_backtrace_sw64(PCONTEXT ctx)
{
	static char filename[MAX_PATH];
	static DWORD64 pcs[MAX_STACK_FRAMES];
	static unsigned num_stack_frames;
	static STACKFRAME64 sf;
	static HANDLE process, thread;
	static DWORD machine;

	process = GetCurrentProcess();
	thread = GetCurrentThread();

	mutex_enter(&backtrace_lock);

	SymInitialize(process, NULL, TRUE);
	SymSetOptions(SYMOPT_LOAD_LINES);

	gather_module_info();

	memset(&sf, 0, sizeof (sf));
	sf.AddrPC.Mode = AddrModeFlat;
	sf.AddrStack.Mode = AddrModeFlat;
	sf.AddrFrame.Mode = AddrModeFlat;
#if	defined(_M_IX86)
	machine = IMAGE_FILE_MACHINE_I386;
	sf.AddrPC.Offset = ctx->Eip;
	sf.AddrStack.Offset = ctx->Esp;
	sf.AddrFrame.Offset = ctx->Ebp;
#elif	defined(_M_X64)
	machine = IMAGE_FILE_MACHINE_AMD64;
	sf.AddrPC.Offset = ctx->Rip;
	sf.AddrStack.Offset = ctx->Rsp;
	sf.AddrFrame.Offset = ctx->Rbp;
#elif	defined(_M_IA64)
	machine = IMAGE_FILE_MACHINE_IA64;
	sf.AddrPC.Offset = ctx->StIIP;
	sf.AddrFrame.Offset = ctx->IntSp;
	sf.AddrBStore.Offset = ctx->RsBSP;
	sf.AddrBStore.Mode = AddrModeFlat;
	sf.AddrStack.Offset = ctx->IntSp;
#else
#error	"Unsupported architecture"
#endif	/* _M_X64 */

	for (num_stack_frames = 0; num_stack_frames < MAX_STACK_FRAMES;
	    num_stack_frames++) {
		if (!StackWalk64(machine, process, thread, &sf, ctx, NULL,
		    SymFunctionTableAccess64, SymGetModuleBase64, NULL)) {
			break;
		}
		pcs[num_stack_frames] = sf.AddrPC.Offset;
	}

	backtrace_buf[0] = '\0';
	lacf_strlcpy(backtrace_buf, BACKTRACE_STR, sizeof (backtrace_buf));

	for (unsigned i = 0; i < num_stack_frames; i++) {
		static int fill;
		static DWORD64 pc;
		static char symname[SYMNAME_MAXLEN + 1];
		static HMODULE module;
		static DWORD64 mbase;

		fill = strlen(backtrace_buf);
		pc = pcs[i];

		module = find_module((LPVOID)pc, &mbase);
		GetModuleFileNameA(module, filename, sizeof (filename));
		find_symbol(filename, (void *)(pc - mbase),
		    symname, sizeof (symname));
		fill += snprintf(&backtrace_buf[fill],
		    sizeof (backtrace_buf) - fill,
		    "%d %p %s+%p (%s)\n", i, (void *)pc, filename,
		    (void *)(pc - mbase), symname);
	}

	if (log_func == NULL)
		abort();
	log_func(backtrace_buf);
	fputs(backtrace_buf, stderr);
	fflush(stderr);
	SymCleanup(process);

	mutex_exit(&backtrace_lock);
}

#else	/* !IBM */

void
log_backtrace(int skip_frames)
{
	static char *msg;
	static size_t msg_len;
	static void *trace[MAX_STACK_FRAMES];
	static size_t i, j, sz;
	static char **fnames;

	sz = backtrace(trace, MAX_STACK_FRAMES);
	fnames = backtrace_symbols(trace, sz);

	for (i = 1 + skip_frames, msg_len = BACKTRACE_STRLEN; i < sz; i++)
		msg_len += snprintf(NULL, 0, "%s\n", fnames[i]);

	msg = (char *)malloc(msg_len + 1);
	strcpy(msg, BACKTRACE_STR);
	for (i = 1 + skip_frames, j = BACKTRACE_STRLEN; i < sz; i++)
		j += sprintf(&msg[j], "%s\n", fnames[i]);

	if (log_func == NULL)
		abort();
	log_func(msg);
	fputs(msg, stderr);

	free(msg);
	free(fnames);
}

#endif	/* !IBM */

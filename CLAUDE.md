# TinyModbus Project Notes

## Toolchain

Always use the project-local AVR toolchain in `avr/bin/` (e.g. `avr/bin/avr-gcc`, `avr/bin/avr-nm`, `avr/bin/avr-size`). Never use system-installed AVR tools from `/opt/homebrew` or anywhere else - they are out of date and produce incorrect results for this project.

The Makefile already sets `PATH := $(shell pwd)/$(AVR_GCC_DIR)/bin:$(PATH)` so `make build` uses the correct tools automatically. For manual invocations, prefix with `avr/bin/` or set PATH accordingly.

## RAM budget

ATtiny1604 has 1KB SRAM. Static data (BSS + data + COMMON) must stay well under 1KB to leave room for the stack (~150 bytes minimum). Key consumers:

- `g_regs` (registers.c): ~672 bytes - the FC03/FC04 register map for 64 ballasts. Includes a 14 B `_reserved` padding hole at 0x0109-0x010F to anchor `group_mask` at the master-visible FC03 address 0x0110 across the FC04 trim (subtask 8).
- `sync_flags[64]` + `current_op` + `group_write` + `commission_state` + `s_active` (dali_sync.c): the sync-flag-driven work bookkeeping that replaced the old `cmd_queue` / `bg_tasks` state.
- `buf` (modbus.c): 40 bytes (`MAX_PKT_SZ = 16*2+8`), limits max FC03/FC04 read to 16 registers (`MAX_REG_RD`).

Packed enums (`__attribute__((packed))`) on `cmd_type_t`, `reg_target_t`, `dali_op_t`, `commission_phase_t`, `dali_kind_t` force 1-byte storage instead of the default 2-byte int-sized AVR enum. The Modbus version string (FC04 at 0x0200) is built on demand directly into `buf` rather than kept in a static buffer.

## Diagnostics

Clang (macOS) will report false-positive errors for AVR-specific attributes (`__progmem__`, `__AVR_TINY_PM_BASE_ADDRESS__`, ISR attributes, EEPROM macros). These are harmless - the code targets avr-gcc, not clang.


## Tasks

- the user will format tasks for claude to execute in the `tasks` directrory
- When executing tasks, create an agent team to process them more effectively
- Tear down team member agents and explicitly kill them when they are no longer needed
- For every coder team member, also fire up a reviewer, who works with the coder to review changes and make suggestions.  The reviewer's job is to make sure that what they write works, is in line with the task, and doesn't drift too far from what the main intent of the system is
- Tasks are performed on a git branch.  your team may check work into this branch at sensible checkpoints. For examle, if the work is broken down into sub-tasks, check in as each subtask is completed, with description.
- When tasks are completed and signed off by the user, move the task to a `done` subdirectory of the tasks directy.  Mark the work as done, and make sure the document is updated with what was changed during the change.  Check any changes into git, and make sure the comment includes informaiton on what was done.  The branch should be ready for a pull request at this time.
- We store our code on a forgejo repostiory.  Use the forgejo mcp tool to create pull requests


## Developer environment
- the user's machine has Tmux installed, and they will be running claude from tmux.  Make appropriate use of tmux panes where necessary
 

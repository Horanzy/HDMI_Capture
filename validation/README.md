# Validation evidence index

`result.json == "passed"` means the Gowin flow reached a bitstream without a
tool-level error. It does not mean DDR calibration, hardware throughput, or
end-to-end capture has been validated on a board.

| Directory | Status | Notes |
|---|---|---|
| `packed422_board_est/` | valid main-branch baseline | Fixed packed YUV422, no DDR3. Evidence for the `main` candidate. |
| `ddr3_mdrp_release/` | valid intermediate baseline | Same DDR RTL as the selected build but with the original 148.5/100 MHz constraints. Kept for the P&R comparison and audit trail. |
| `ddr3_mdrp_over5/` | **selected DDR candidate** | Deliberate video +1.4% / DDR +5% P&R overconstraint; zero setup/hold violations. Actual hardware clocks are unchanged. Start with `comparison.txt`. |

The former `ddr3_bringup/` and `ddr3_800_pipelined/` directories were removed
from the current tree because they described superseded or invalid
configurations (swept `PLL_INIT`, 50 MHz UI, and the old 200 MB/s command
ceiling). They remain recoverable from Git history, but must not be used as
current evidence.

Do not delete the checked-in Gowin IP triplets (`.ipc`, generated `.v`,
`*_tmp.v`) or the DDR PHY simulation netlist `.vo`: current tool versions
cannot reliably regenerate the legacy external-mDRP PLL configuration.

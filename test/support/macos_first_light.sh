#!/usr/bin/env bash
# First light on macOS (Apple Silicon): take a new Mac from nothing to a working
# SpaceAGORAMuJoCo environment.
#
# It checks the machine, finds or installs Julia 1.12, runs scripts/setup_env.jl
# (which installs the Julia packages and builds the MuJoCo 3.11.0 artifact from
# the official disk image), and confirms that the native library loads. It asks
# before it installs anything. Run test/support/macos_shakedown.sh afterwards to
# test the result.
#
# Usage: test/support/macos_first_light.sh [--env DIR] [--yes] [--no-color] [--help]

set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
DEFAULT_ENV="$(dirname "$REPO")/mujoco-env"
ENV_DIR="${SPACEAGORA_MUJOCO_ENV:-}"
ASSUME_YES=0
USE_COLOR=1
TOTAL_STEPS=6

usage() {
    cat <<EOF
First light: set up SpaceAGORAMuJoCo on macOS (Apple Silicon) for the first time.

Usage: test/support/macos_first_light.sh [options]

  --env DIR    Environment directory to create or reuse.
               Default: $DEFAULT_ENV
  --yes, -y    Do not ask; accept the default answer to every question.
               The defaults install Julia if it is missing.
  --no-color   Plain output.
  --help, -h   Show this help.

The environment directory must be outside this checkout. A directory that this
installer created earlier is reused; any other nonempty directory is refused.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --env) [ $# -ge 2 ] || { echo "--env needs a directory" >&2; exit 2; }; ENV_DIR="$2"; shift 2 ;;
        --env=*) ENV_DIR="${1#--env=}"; shift ;;
        --yes|-y) ASSUME_YES=1; shift ;;
        --no-color) USE_COLOR=0; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

# ---------------------------------------------------------------- appearance
if [ ! -t 1 ] || [ -n "${NO_COLOR:-}" ]; then USE_COLOR=0; fi
if [ "$USE_COLOR" = 1 ]; then
    BOLD=$'\033[1m'; DIM=$'\033[2m'; RESET=$'\033[0m'
    RED=$'\033[31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; BLUE=$'\033[34m'; MAGENTA=$'\033[35m'; CYAN=$'\033[36m'
else
    BOLD=""; DIM=""; RESET=""; RED=""; GREEN=""; YELLOW=""; BLUE=""; MAGENTA=""; CYAN=""
fi

STEP=0
step()  { STEP=$((STEP + 1)); printf '\n%s━━ Step %d of %d ━ %s%s\n' "$BOLD$BLUE" "$STEP" "$TOTAL_STEPS" "$1" "$RESET"; }
ok()    { printf '  %s✔%s %s\n' "$GREEN" "$RESET" "$1"; }
info()  { printf '  %s•%s %s\n' "$CYAN" "$RESET" "$1"; }
warn()  { printf '  %s!%s %s\n' "$YELLOW" "$RESET" "$1"; }
note()  { printf '    %s%s%s\n' "$DIM" "$1" "$RESET"; }
shown() { printf '    %s$ %s%s\n' "$MAGENTA" "$1" "$RESET"; }
die()   { printf '\n  %s✘ %s%s\n' "$RED$BOLD" "$1" "$RESET" >&2; shift
          for line in "$@"; do printf '    %s\n' "$line" >&2; done; printf '\n' >&2; exit 1; }

# ask "question" default(y|n): returns 0 for yes. Without a terminal, or with --yes, the default answers.
ask() {
    local question="$1" default="$2" hint answer
    if [ "$default" = y ]; then hint="Y/n"; else hint="y/N"; fi
    if [ "$ASSUME_YES" = 1 ] || [ ! -t 0 ]; then
        printf '  %s?%s %s %s[%s] %s%s\n' "$YELLOW$BOLD" "$RESET" "$question" "$DIM" "$hint" "$default (not asked)" "$RESET"
        [ "$default" = y ]; return
    fi
    while true; do
        printf '  %s?%s %s %s[%s]%s ' "$YELLOW$BOLD" "$RESET" "$question" "$DIM" "$hint" "$RESET"
        read -r answer || answer=""
        case "$(printf '%s' "${answer:-$default}" | tr '[:upper:]' '[:lower:]')" in
            y|yes) return 0 ;;
            n|no) return 1 ;;
            *) note "Please answer y or n." ;;
        esac
    done
}

elapsed() { local s=$1; if [ "$s" -ge 60 ]; then printf '%dm %02ds' $((s / 60)) $((s % 60)); else printf '%ds' "$s"; fi; }

printf '\n%s' "$BOLD$CYAN"
cat <<'EOF'
   ╭──────────────────────────────────────────────────────────────╮
   │   F I R S T   L I G H T                                      │
   │   SpaceAGORAMuJoCo on macOS · Apple Silicon · first setup    │
   ╰──────────────────────────────────────────────────────────────╯
EOF
printf '%s' "$RESET"
info "This script prepares everything a first run needs:"
note "1. checks this Mac            4. installs the Julia packages"
note "2. finds or installs Julia    5. builds the MuJoCo 3.11.0 artifact"
note "3. chooses an environment     6. confirms the native library loads"
info "It asks before installing anything. Nothing is committed or pushed."
info "Checkout: $REPO"

# ---------------------------------------------------------------- 1. machine
step "Check this Mac"
[ "$(uname -s)" = Darwin ] || die "This script is for macOS." \
    "On Linux x86_64 use the commands in README.md under Installation."
ok "macOS $(sw_vers -productVersion 2>/dev/null || echo '?')"

if [ "$(sysctl -n hw.optional.arm64 2>/dev/null || echo 0)" != 1 ]; then
    die "This Mac has an Intel processor, which is not supported." \
        "Native scenes need Apple Silicon (arm64) or Linux x86_64."
fi
if [ "$(uname -m)" != arm64 ]; then
    die "This terminal runs under Rosetta (x86_64 emulation)." \
        "MuJoCo cannot run under Rosetta. Open a native terminal, or run:" \
        "  arch -arm64 $0"
fi
ok "Apple Silicon, native arm64 terminal"

if ! xcode-select -p >/dev/null 2>&1 || ! command -v git >/dev/null 2>&1; then
    warn "Git is missing. It comes with the Xcode Command Line Tools."
    if ask "Open the Command Line Tools installer now?" y; then
        xcode-select --install >/dev/null 2>&1 || true
        die "Finish the installer window that opened, then run this script again."
    fi
    die "Git is required." "Install it with: xcode-select --install"
fi
ok "$(git --version)"

command -v hdiutil >/dev/null 2>&1 || die "hdiutil was not found." "It is part of macOS and is needed to open the MuJoCo disk image."
ok "hdiutil is available"

[ -f "$REPO/scripts/setup_env.jl" ] && [ -f "$REPO/NativeLibrary.toml" ] ||
    die "This does not look like a SpaceAGORAMuJoCo checkout with macOS support." "Looked in: $REPO"
ok "Checkout has the installer and NativeLibrary.toml"

# ---------------------------------------------------------------- 2. Julia
step "Find Julia 1.12"
JULIA=()
julia_is_112() { "$@" --startup-file=no -e 'exit(VERSION.major == 1 && VERSION.minor == 12 ? 0 : 1)' >/dev/null 2>&1; }

find_julia() {
    JULIA=()
    if command -v julia >/dev/null 2>&1 && julia_is_112 julia; then JULIA=(julia); return 0; fi
    if command -v juliaup >/dev/null 2>&1 && command -v julia >/dev/null 2>&1 && julia_is_112 julia +1.12; then
        JULIA=(julia +1.12); return 0
    fi
    return 1
}

if ! find_julia; then
    export PATH="$HOME/.juliaup/bin:$PATH"   # where the official installer puts julia and juliaup
    if command -v juliaup >/dev/null 2>&1; then
        warn "juliaup is installed, but Julia 1.12 is not."
        ask "Add Julia 1.12 with juliaup? Your default Julia version stays as it is." y ||
            die "Julia 1.12 is required." "Add it with: juliaup add 1.12"
        shown "juliaup add 1.12"
        juliaup add 1.12 || die "juliaup could not add Julia 1.12."
    else
        warn "Julia is not installed."
        note "juliaup is the official Julia installer and version manager."
        if command -v brew >/dev/null 2>&1; then
            ask "Install juliaup with Homebrew, then Julia 1.12?" y ||
                die "Julia 1.12 is required." "Install it with: brew install juliaup && juliaup add 1.12"
            shown "brew install juliaup"
            brew install juliaup || die "Homebrew could not install juliaup."
        else
            ask "Install juliaup with the official installer (https://install.julialang.org), then Julia 1.12?" y ||
                die "Julia 1.12 is required." "See https://julialang.org/install/"
            shown "curl -fsSL https://install.julialang.org | sh -s -- --yes"
            curl -fsSL https://install.julialang.org | sh -s -- --yes || die "The Julia installer failed."
        fi
        shown "juliaup add 1.12"
        juliaup add 1.12 || die "juliaup could not add Julia 1.12."
    fi
    hash -r
    find_julia || die "Julia 1.12 is still not usable from this terminal." \
        "Open a new terminal window and run this script again."
fi
JULIA_TEXT="${JULIA[*]}"
ok "$("${JULIA[@]}" --version) (command: $JULIA_TEXT)"
JULIA_ARCH="$("${JULIA[@]}" --startup-file=no -e 'print(Sys.ARCH)')"
[ "$JULIA_ARCH" = aarch64 ] || die "This Julia is an $JULIA_ARCH build; a native aarch64 build is needed." \
    "With juliaup: juliaup add 1.12~aarch64"
ok "Julia is a native aarch64 build"

# ---------------------------------------------------------------- 3. environment
step "Choose the environment directory"
info "The environment holds the Julia package versions for this checkout."
note "It must be outside the checkout, and it must stay where it is: it refers"
note "to this checkout and to its own sources/ directory by path."
if [ -z "$ENV_DIR" ]; then
    ENV_DIR="$DEFAULT_ENV"
    if [ "$ASSUME_YES" = 0 ] && [ -t 0 ]; then
        printf '  %s?%s Environment directory %s[Enter for %s]%s ' "$YELLOW$BOLD" "$RESET" "$DIM" "$DEFAULT_ENV" "$RESET"
        read -r reply || reply=""
        [ -n "$reply" ] && ENV_DIR="$reply"
    fi
fi
case "$ENV_DIR" in "~"*) ENV_DIR="$HOME${ENV_DIR#\~}" ;; esac
case "$ENV_DIR" in /*) ;; *) ENV_DIR="$PWD/$ENV_DIR" ;; esac
case "$ENV_DIR/" in "$REPO"/*) die "The environment cannot be inside the checkout." "Choose a directory outside $REPO" ;; esac

MARKER="$ENV_DIR/.spaceagora-mujoco-environment.toml"
if [ -f "$MARKER" ]; then
    ok "Reusing the environment at $ENV_DIR"
    note "The installer created it earlier; it will be brought up to date."
elif [ -d "$ENV_DIR" ] && [ -n "$(ls -A "$ENV_DIR" 2>/dev/null)" ]; then
    die "$ENV_DIR exists and was not created by this installer." "Choose an empty or new directory with --env DIR."
else
    ok "A new environment will be created at $ENV_DIR"
fi

if [ -n "${SPACEAGORA_CORE_PATH:-}" ] || [ -n "${SPACEAGORA_CORE_REV:-}" ]; then
    warn "SPACEAGORA_CORE_PATH or SPACEAGORA_CORE_REV is set in this terminal."
    note "They replace the SpaceAGORA revision this checkout pins. A first setup should use the pin."
    if ask "Ignore them for this run and use the pinned SpaceAGORA?" y; then
        unset SPACEAGORA_CORE_PATH SPACEAGORA_CORE_REV
        ok "Using the pinned SpaceAGORA revision"
    else
        warn "Keeping the override; this is a compatibility probe, not the supported pair."
    fi
fi

# ---------------------------------------------------------------- 4 and 5. install
step "Install the Julia packages"
info "This runs the repository's installer:"
shown "$JULIA_TEXT --startup-file=no scripts/setup_env.jl $ENV_DIR"
note "It clones the pinned SpaceAGORA, resolves about 300 packages and precompiles them."
note "A first run takes roughly 5 to 10 minutes and needs the network. Reruns take under a minute."
note "Then it downloads the official MuJoCo disk image (46 MB), checks its sha256,"
note "copies mujoco.framework into a Julia artifact and detaches the image."
ask "Start the installation?" y || die "Nothing was installed." "Run this script again when you are ready."

START=$SECONDS
SETUP_LOG="$(mktemp "${TMPDIR:-/tmp}/spaceagora-first-light.XXXXXX")"
(cd "$REPO" && "${JULIA[@]}" --startup-file=no scripts/setup_env.jl "$ENV_DIR" 2>&1) |
    tee "$SETUP_LOG" | sed "s/^/    ${DIM}│${RESET} /"
SETUP_STATUS=${PIPESTATUS[0]}
if [ "$SETUP_STATUS" != 0 ]; then
    die "The installer failed after $(elapsed $((SECONDS - START)))." \
        "The full output is in $SETUP_LOG" \
        "Common causes: no network, or an interrupted download. It is safe to run this script again."
fi
ok "Packages installed in $(elapsed $((SECONDS - START)))"

step "Check the MuJoCo artifact"
if grep -q "Native library: MuJoCo 3011000 is installed and loads" "$SETUP_LOG"; then
    ok "The installer built or found the MuJoCo 3.11.0 artifact and loaded it"
else
    die "The installer finished without reporting the native library." "See $SETUP_LOG"
fi
if hdiutil info 2>/dev/null | grep -qi mujoco; then
    warn "A MuJoCo disk image is still mounted. List it with 'hdiutil info' and release it with 'hdiutil detach <mount point>'."
else
    ok "No MuJoCo disk image is left mounted"
fi
rm -f "$SETUP_LOG"

# ---------------------------------------------------------------- 6. verify
step "Confirm the native library loads"
export JULIA_LOAD_PATH='@:@stdlib'
VERIFY='using SpaceAGORA, SpaceAGORAMuJoCo; B = SpaceAGORAMuJoCo.Binding; B.version() == 3011000 || error("unexpected MuJoCo version"); println(B.library_path())'
if LIBRARY="$("${JULIA[@]}" --startup-file=no --project="$ENV_DIR" -e "$VERIFY" 2>&1)"; then
    ok "MuJoCo 3.11.0 loads in a new Julia process"
    note "$(printf '%s' "$LIBRARY" | tail -n 1)"
else
    printf '%s\n' "$LIBRARY" | sed 's/^/    /' >&2
    die "The packages are installed, but the native library did not load."
fi

printf '\n%s   ✔ First light: the environment is ready.%s\n\n' "$GREEN$BOLD" "$RESET"
info "Start Julia in it:"
shown "JULIA_LOAD_PATH='@:@stdlib' $JULIA_TEXT --startup-file=no --project=$ENV_DIR"
info "Then, in Julia:"
shown "using SpaceAGORA, SpaceAGORAMuJoCo"
info "The user guide's first example is in docs/user_guide.md."
info "Keep these three together; the environment refers to them by path:"
note "$REPO"
note "$ENV_DIR"
note "$ENV_DIR/sources"
printf '\n'

SHAKEDOWN="$REPO/test/support/macos_shakedown.sh"
if [ -x "$SHAKEDOWN" ] && ask "Run the shakedown tests now? (about 4 minutes)" y; then
    ARGS=(--env "$ENV_DIR")
    [ "$ASSUME_YES" = 1 ] && ARGS+=(--yes)
    [ "$USE_COLOR" = 0 ] && ARGS+=(--no-color)
    exec "$SHAKEDOWN" "${ARGS[@]}"
fi
info "Test the installation later with:"
shown "test/support/macos_shakedown.sh --env $ENV_DIR"
printf '\n'

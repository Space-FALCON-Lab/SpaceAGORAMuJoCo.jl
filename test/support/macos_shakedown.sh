#!/usr/bin/env bash
# Shakedown on macOS (Apple Silicon): test an environment that
# test/support/macos_first_light.sh (or scripts/setup_env.jl) created.
#
# Stages:
#   1. Preflight        Julia 1.12, the environment, the native library
#   2. Quick checks     manifests and installer logic; no simulation
#   3. Isolation        core-only import, refusals, both load orders
#   4. Native suite     the full test suite with four threads (about 2 minutes)
#   5. First example    the user guide's two-cube scene
#   6. Fresh install    optional, asked: rebuild the MuJoCo artifact from the
#                       official disk image in a throwaway environment
#
# Usage: test/support/macos_shakedown.sh [--env DIR] [--yes] [--full] [--quick] [--no-color] [--help]

set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
DEFAULT_ENV="$(dirname "$REPO")/mujoco-env"
ENV_DIR="${SPACEAGORA_MUJOCO_ENV:-}"
ASSUME_YES=0
FULL=0
QUICK=0
USE_COLOR=1

usage() {
    cat <<EOF
Shakedown: test a SpaceAGORAMuJoCo environment on macOS (Apple Silicon).

Usage: test/support/macos_shakedown.sh [options]

  --env DIR    Environment to test. Default: $DEFAULT_ENV
  --yes, -y    Do not ask; accept the default answer to every question.
               The optional fresh-install test is skipped unless --full is given.
  --full       Also run the optional fresh-install test without asking.
  --quick      Skip the native suite (stage 4) and the fresh-install test.
  --no-color   Plain output.
  --help, -h   Show this help.

The exit status is 0 when every check that ran passed.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --env) [ $# -ge 2 ] || { echo "--env needs a directory" >&2; exit 2; }; ENV_DIR="$2"; shift 2 ;;
        --env=*) ENV_DIR="${1#--env=}"; shift ;;
        --yes|-y) ASSUME_YES=1; shift ;;
        --full) FULL=1; shift ;;
        --quick) QUICK=1; shift ;;
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

stage() { printf '\n%s━━ Stage %s ━ %s%s\n' "$BOLD$BLUE" "$1" "$2" "$RESET"; }
ok()    { printf '  %s✔%s %s\n' "$GREEN" "$RESET" "$1"; }
info()  { printf '  %s•%s %s\n' "$CYAN" "$RESET" "$1"; }
warn()  { printf '  %s!%s %s\n' "$YELLOW" "$RESET" "$1"; }
note()  { printf '    %s%s%s\n' "$DIM" "$1" "$RESET"; }
shown() { printf '    %s$ %s%s\n' "$MAGENTA" "$1" "$RESET"; }
die()   { printf '\n  %s✘ %s%s\n' "$RED$BOLD" "$1" "$RESET" >&2; shift
          for line in "$@"; do printf '    %s\n' "$line" >&2; done; printf '\n' >&2; exit 1; }

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

# ---------------------------------------------------------------- running checks
LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/spaceagora-shakedown.XXXXXX")"
RESULT_NAMES=(); RESULT_STATES=(); RESULT_TIMES=()
CHECKS=0; FAILURES=0; LAST_LOG=""
SPINNER=(⠋ ⠙ ⠹ ⠸ ⠼ ⠴ ⠦ ⠧ ⠇ ⠏)
KEEP_LOGS=0

record() { RESULT_NAMES+=("$1"); RESULT_STATES+=("$2"); RESULT_TIMES+=("$3"); }

# The line under the last "Test Summary:" header of a log, with its columns squeezed: name | counts.
summary_of() { awk '/^Test Summary:/ { getline; line = $0 } END { print line }' "$1" | tr -s ' ' | sed 's/ *$//'; }

# check "name" "what it shows" command...   Runs in the checkout; output goes to a log, shown only on failure.
check() {
    local name="$1" purpose="$2"; shift 2
    CHECKS=$((CHECKS + 1))
    LAST_LOG="$LOG_DIR/$(printf '%02d' "$CHECKS")-$(printf '%s' "$name" | tr -c 'A-Za-z0-9' '_').log"
    local start=$SECONDS status=0 frame=0 pid
    (cd "$REPO" && "$@") >"$LAST_LOG" 2>&1 &
    pid=$!
    if [ -t 1 ]; then
        while kill -0 "$pid" 2>/dev/null; do
            printf '\r  %s%s%s %s %s(%s)%s ' "$CYAN" "${SPINNER[$((frame % 10))]}" "$RESET" "$name" "$DIM" "$(elapsed $((SECONDS - start)))" "$RESET"
            frame=$((frame + 1)); sleep 0.1
        done
        printf '\r\033[K'
    else
        printf '  … %s\n' "$name"
    fi
    wait "$pid" || status=$?
    local took; took="$(elapsed $((SECONDS - start)))"
    if [ "$status" = 0 ]; then
        printf '  %s✔%s %s %s(%s)%s\n' "$GREEN" "$RESET" "$name" "$DIM" "$took" "$RESET"
        note "$purpose"
        local summary; summary="$(summary_of "$LAST_LOG")"
        [ -n "$summary" ] && note "↳ $summary"
        record "$name" pass "$took"
    else
        FAILURES=$((FAILURES + 1))
        printf '  %s✘ %s%s %s(%s, exit %s)%s\n' "$RED$BOLD" "$name" "$RESET" "$DIM" "$took" "$status" "$RESET"
        note "$purpose"
        printf '    %s── last lines of the output ──%s\n' "$RED" "$RESET"
        tail -n 25 "$LAST_LOG" | sed "s/^/    ${RED}│${RESET} /"
        note "Full output: $LAST_LOG"
        record "$name" FAIL "$took"
    fi
    return "$status"
}

skip() { printf '  %s–%s %s %s(skipped: %s)%s\n' "$YELLOW" "$RESET" "$1" "$DIM" "$2" "$RESET"; record "$1" skip "-"; }

printf '\n%s' "$BOLD$CYAN"
cat <<'EOF'
   ╭──────────────────────────────────────────────────────────────╮
   │   S H A K E D O W N                                          │
   │   SpaceAGORAMuJoCo on macOS · Apple Silicon · test run       │
   ╰──────────────────────────────────────────────────────────────╯
EOF
printf '%s' "$RESET"
info "Checkout: $REPO"
info "Each check prints ✔ or ✘. Output is shown only when a check fails."
info "Logs: $LOG_DIR"

# ---------------------------------------------------------------- 1. preflight
stage "1 of 6" "Preflight"
[ "$(uname -s)" = Darwin ] && [ "$(uname -m)" = arm64 ] ||
    die "This script is for macOS on Apple Silicon, in a native arm64 terminal." "Found: $(uname -s) $(uname -m)"
ok "macOS $(sw_vers -productVersion 2>/dev/null || echo '?'), arm64"

export PATH="$PATH:$HOME/.juliaup/bin"
JULIA=()
julia_is_112() { "$@" --startup-file=no -e 'exit(VERSION.major == 1 && VERSION.minor == 12 ? 0 : 1)' >/dev/null 2>&1; }
if command -v julia >/dev/null 2>&1 && julia_is_112 julia; then
    JULIA=(julia)
elif command -v juliaup >/dev/null 2>&1 && command -v julia >/dev/null 2>&1 && julia_is_112 julia +1.12; then
    JULIA=(julia +1.12)
else
    die "Julia 1.12 was not found." "Run test/support/macos_first_light.sh first; it installs Julia and the environment."
fi
JULIA_TEXT="${JULIA[*]}"
ok "$("${JULIA[@]}" --version) (command: $JULIA_TEXT)"

if [ -z "$ENV_DIR" ]; then
    ENV_DIR="$DEFAULT_ENV"
    if [ "$ASSUME_YES" = 0 ] && [ -t 0 ]; then
        printf '  %s?%s Environment to test %s[Enter for %s]%s ' "$YELLOW$BOLD" "$RESET" "$DIM" "$DEFAULT_ENV" "$RESET"
        read -r reply || reply=""
        [ -n "$reply" ] && ENV_DIR="$reply"
    fi
fi
case "$ENV_DIR" in "~"*) ENV_DIR="$HOME${ENV_DIR#\~}" ;; esac
case "$ENV_DIR" in /*) ;; *) ENV_DIR="$PWD/$ENV_DIR" ;; esac
[ -f "$ENV_DIR/Manifest.toml" ] && [ -f "$ENV_DIR/.spaceagora-mujoco-environment.toml" ] ||
    die "$ENV_DIR is not an installed SpaceAGORAMuJoCo environment." \
        "Create one with: test/support/macos_first_light.sh --env $ENV_DIR"
ok "Environment: $ENV_DIR"
if ! grep -q "$REPO" "$ENV_DIR/.spaceagora-mujoco-environment.toml"; then
    warn "This environment was created from a different checkout; the tests below run this checkout's files against it."
fi

# The settings the CI job uses.
export JULIA_LOAD_PATH='@:@stdlib' JULIA_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 JULIA_PKG_PRECOMPILE_AUTO=0
export SPACEAGORA_MUJOCO_REQUIRED=1
unset SPACEAGORA_CORE_PATH SPACEAGORA_CORE_REV
RUN=("${JULIA[@]}" --startup-file=no)
IN_ENV=("${JULIA[@]}" --startup-file=no "--project=$ENV_DIR")

# Test runs leave these in the checkout; remove them at the end only if they were not here before.
HAD_OUTPUT=0; [ -e "$REPO/output" ] && HAD_OUTPUT=1
HAD_MJLOG=0; [ -e "$REPO/MUJOCO_LOG.TXT" ] && HAD_MJLOG=1

if ! check "Native library loads" "MuJoCo 3.11.0 loads from the installed artifact (mj_version is 3011000)." \
        "${IN_ENV[@]}" -e 'using SpaceAGORA, SpaceAGORAMuJoCo; B = SpaceAGORAMuJoCo.Binding; B.version() == 3011000 || error("unexpected MuJoCo version"); println(B.library_path())'; then
    die "The native library does not load, so the remaining checks cannot run." \
        "Repair the environment with: test/support/macos_first_light.sh --env $ENV_DIR"
fi
note "↳ $(tail -n 1 "$LAST_LOG")"

# ---------------------------------------------------------------- 2. quick checks
stage "2 of 6" "Quick checks (no simulation, a few seconds each)"
check "Native library manifests" "NativeLibrary.toml and Artifacts.toml agree; setup prefetches on macOS and stays lazy on Linux." \
    "${RUN[@]}" test/native_library_tests.jl
check "Installation plan" "The installer plans the pinned SpaceAGORA and refuses unsafe destinations." \
    "${RUN[@]}" test/installation_plan_tests.jl
check "Source acquisition" "The installer accepts only the exact pinned core commit (uses local Git fixtures)." \
    "${RUN[@]}" test/source_acquisition_tests.jl

# ---------------------------------------------------------------- 3. isolation
stage "3 of 6" "Isolation, refusals and load order"
check "Core-only import" "SpaceAGORA alone loads without the companion or MuJoCo." \
    "${IN_ENV[@]}" test/core_only.jl
check "Incompatible core refused" "An incompatible service contract is refused before the adapter is defined." \
    "${IN_ENV[@]}" test/incompatible_core.jl
check "Load order: core first" "Loading SpaceAGORA, then the companion, registers the adapter without loading MuJoCo." \
    "${IN_ENV[@]}" test/load_order.jl core-first
check "Load order: companion first" "The opposite order gives the same result." \
    "${IN_ENV[@]}" test/load_order.jl companion-first
if check "Missing library refused" "With the artifact hidden, a scene is refused with a clear error and nothing is downloaded." \
        "${IN_ENV[@]}" test/missing_native.jl; then
    note "↳ $(grep -m 1 '^REFUSED:' "$LAST_LOG" | cut -c 1-110)"
fi

# ---------------------------------------------------------------- 4. native suite
stage "4 of 6" "Native suite (four threads, about 2 minutes)"
if [ "$QUICK" = 1 ]; then
    skip "Native test suite" "--quick"
elif check "Native test suite" "Offsets against MuJoCo readbacks, orbital agreement, attitude conventions, coupling, threaded Monte Carlo." \
        "${IN_ENV[@]}" -e '@assert Threads.nthreads() == 4; include("test/runtests.jl")'; then
    info "On macOS, \"2 Broken\" is expected: two memory bounds read glibc's mallinfo2, which macOS lacks."
    info "The \"Nan, Inf or huge value in QACC\" warning in the log is provoked on purpose by one test."
fi

# ---------------------------------------------------------------- 5. example
stage "5 of 6" "First example"
EXAMPLE="$LOG_DIR/two_cubes.jl"
cat >"$EXAMPLE" <<'EOF'
# The standalone example of docs/user_guide.md: two cubes 20 m apart at a 7000 km radius, 500 steps of 0.02 s.
using SpaceAGORA, SpaceAGORAMuJoCo
using StaticArrays, LinearAlgebra

xml = """
<mujoco model="two orbital cubes">
  <compiler angle="radian"/>
  <worldbody>
    <body name="chaser">
      <freejoint/>
      <inertial pos="0 0 0" mass="12" diaginertia="2 2 2"/>
      <geom type="box" size="0.5 0.5 0.5" mass="0" contype="0" conaffinity="0"/>
    </body>
    <body name="target">
      <freejoint/>
      <inertial pos="0 0 0" mass="12" diaginertia="2 2 2"/>
      <geom type="box" size="0.5 0.5 0.5" mass="0" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>
"""

earth = SpaceAGORA.make_no_gram_planet(:earth)
gravity = (SpaceAGORA.InverseSquaredGravityModel(),)
r1 = SVector(7.0e6, 0.0, 0.0)
v1 = SVector(0.0, sqrt(earth.μ / norm(r1)), 0.0)
r2 = r1 + SVector(0.0, 20.0, 0.0)
initial_states = [SceneBodyState("chaser", r1, v1), SceneBodyState("target", r2, v1)]

scene = ProximityScene(; mjcf_xml=xml, dt=0.02, planet=earth,
    gravity_effectors=gravity, initial_states=initial_states)
for _ in 1:500
    scene_step!(scene)
end

chaser = scene_body_state(scene, "chaser")
target = scene_body_state(scene, "target")
separation = norm(target.r - chaser.r)
println("Elapsed time: ", scene_time(scene), " s")
println("Separation: ", separation, " m")
println("Relative velocity: ", target.v - chaser.v, " m/s")
scene_time(scene) ≈ 10.0 || error("expected 10 s of scene time")
19.9 < separation < 20.1 || error("the cubes should still be about 20 m apart")
close(scene)
EOF
if check "Two-cube scene" "A standalone scene steps for 10 s; the cubes stay about 20 m apart." \
        "${IN_ENV[@]}" "$EXAMPLE"; then
    while IFS= read -r line; do note "↳ $line"; done <"$LAST_LOG"
fi

# ---------------------------------------------------------------- 6. optional fresh install
stage "6 of 6" "Fresh install (optional)"
info "This repeats what a brand-new Mac does: it removes the installed MuJoCo artifact,"
note "creates a throwaway environment, downloads the official disk image (46 MB) again,"
note "verifies it, rebuilds the artifact and checks that the image is detached."
note "It takes 1 to 3 minutes and needs the network. Your environment is not modified."
RUN_FRESH=0
if [ "$QUICK" = 1 ]; then
    :
elif [ "$FULL" = 1 ]; then
    RUN_FRESH=1
elif [ "$ASSUME_YES" = 0 ] && ask "Run the fresh-install test?" n; then
    RUN_FRESH=1
fi

if [ "$RUN_FRESH" = 0 ]; then
    skip "Fresh install" "not requested; use --full to include it"
else
    ARTIFACT="$("${IN_ENV[@]}" -e 'using SpaceAGORAMuJoCo; print(dirname(dirname(dirname(dirname(SpaceAGORAMuJoCo.Binding.library_path())))))' 2>/dev/null)"
    FRESH_ENV="$LOG_DIR/fresh-env"
    REMOVED=0
    case "$ARTIFACT" in
        */artifacts/????????????????????????????????????????)
            info "Removing the artifact so that the download and build really happen:"
            shown "rm -rf $ARTIFACT"
            chmod -R u+w "$ARTIFACT" 2>/dev/null; rm -rf "$ARTIFACT"; REMOVED=1 ;;
        *) warn "The artifact directory was not recognised ($ARTIFACT); it is left in place, so setup will reuse it." ;;
    esac
    shown "$JULIA_TEXT --startup-file=no scripts/setup_env.jl $FRESH_ENV"
    if check "Fresh installation" "The installer builds the environment and the MuJoCo artifact from nothing." \
            "${RUN[@]}" scripts/setup_env.jl "$FRESH_ENV"; then
        SETUP_LOG="$LAST_LOG"
        if [ "$REMOVED" = 1 ]; then
            check "Disk image downloaded" "Setup fetched the official disk image again and its sha256 matched." \
                grep -q "downloading the official MuJoCo disk image" "$SETUP_LOG"
        fi
        check "Artifact built and loaded" "Setup reported that the rebuilt MuJoCo 3.11.0 library loads." \
            grep -q "Native library: MuJoCo 3011000 is installed and loads" "$SETUP_LOG"
        check "Disk image detached" "No MuJoCo image is left mounted after setup." \
            sh -c '! hdiutil info | grep -qi mujoco'
        check "Fresh environment loads MuJoCo" "A new process in the fresh environment loads the rebuilt library." \
            "${JULIA[@]}" --startup-file=no "--project=$FRESH_ENV" -e 'using SpaceAGORA, SpaceAGORAMuJoCo; SpaceAGORAMuJoCo.Binding.version() == 3011000 || error("unexpected version")'
        check "Your environment still loads MuJoCo" "The environment under test finds the rebuilt artifact." \
            "${IN_ENV[@]}" -e 'using SpaceAGORAMuJoCo; SpaceAGORAMuJoCo.Binding.version() == 3011000 || error("unexpected version")'
    else
        warn "The artifact was removed and not rebuilt. Restore it with: test/support/macos_first_light.sh --env $ENV_DIR"
    fi
    if [ -d "$FRESH_ENV" ]; then
        if ask "Delete the throwaway environment ($(du -sh "$FRESH_ENV" 2>/dev/null | cut -f 1))?" y; then
            rm -rf "$FRESH_ENV"; ok "Throwaway environment deleted"
        else
            info "Kept: $FRESH_ENV"; KEEP_LOGS=1
        fi
    fi
fi

# ---------------------------------------------------------------- tidy up and report
[ "$HAD_OUTPUT" = 0 ] && rm -rf "$REPO/output"
[ "$HAD_MJLOG" = 0 ] && rm -f "$REPO/MUJOCO_LOG.TXT"

printf '\n%s━━ Summary%s\n' "$BOLD$BLUE" "$RESET"
i=0
while [ "$i" -lt "${#RESULT_NAMES[@]}" ]; do
    case "${RESULT_STATES[$i]}" in
        pass) mark="${GREEN}✔ pass${RESET}" ;;
        skip) mark="${YELLOW}– skip${RESET}" ;;
        *)    mark="${RED}${BOLD}✘ FAIL${RESET}" ;;
    esac
    printf '  %s  %-38s %s%8s%s\n' "$mark" "${RESULT_NAMES[$i]}" "$DIM" "${RESULT_TIMES[$i]}" "$RESET"
    i=$((i + 1))
done
printf '  %sTotal time: %s%s\n' "$DIM" "$(elapsed "$SECONDS")" "$RESET"

if [ "$FAILURES" = 0 ]; then
    printf '\n%s   ✔ Shakedown complete: every check that ran passed.%s\n' "$GREEN$BOLD" "$RESET"
    [ "$KEEP_LOGS" = 0 ] && rm -rf "$LOG_DIR"
    printf '\n'
    exit 0
fi
printf '\n%s   ✘ %d check(s) failed.%s\n' "$RED$BOLD" "$FAILURES" "$RESET"
info "Logs are kept in $LOG_DIR"
info "If you report this, include the failing log and the output of: $JULIA_TEXT --version"
printf '\n'
exit 1

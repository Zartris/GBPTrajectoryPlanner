#!/usr/bin/env bash
# GBPTrajectoryPlanner — tmux cheat sheet
# Printed once on session creation; stays in scroll history (Prefix+[ to scroll up).

BOLD="\033[1m"
DIM="\033[2m"
RESET="\033[0m"
CYAN="\033[36m"
ORANGE="\033[38;5;166m"
WHITE="\033[97m"
GREY="\033[38;5;240m"

BAR="${GREY}│${RESET}"
TOP="${GREY}╔══════════════════════════════════════════════════════╗${RESET}"
MID="${GREY}╠══════════════════════════════════════════════════════╣${RESET}"
DIV="${GREY}╠════════════════╬═════════════════════════════════════╣${RESET}"
BOT="${GREY}╚════════════════╩═════════════════════════════════════╝${RESET}"
HDR="${GREY}║                ${GREY}║${RESET}"

pad()  { printf "%-16s" "$1"; }
row()  { printf "${GREY}║${RESET} ${ORANGE}%-14s${RESET} ${BAR} ${WHITE}%-31s${RESET} ${GREY}║${RESET}\n" "$1" "$2"; }
blank(){ printf "${GREY}║${RESET} %-16s ${BAR} %-31s ${GREY}║${RESET}\n" "" ""; }

echo ""
echo -e "$TOP"
printf "${GREY}║${RESET}  ${ORANGE}${BOLD}GBPTrajectoryPlanner${RESET}  ${GREY}—${RESET}  ${CYAN}tmux quick-ref${RESET}  ${GREY}(prefix = ${ORANGE}Ctrl+A${GREY})${RESET}   ${GREY}║${RESET}\n"
echo -e "$MID"
printf "${GREY}║${RESET}  ${BOLD}${WHITE}%-14s${RESET}  ${BAR}  ${BOLD}${WHITE}%-30s${RESET} ${GREY}║${RESET}\n" "ACTION" "KEYS"
echo -e "$DIV"
row "─── PANES ───" ""
row "Split horizontal" "Prefix + o  (top / bottom)"
row "Split vertical"   "Prefix + e  (left / right)"
row "Navigate"         "Prefix + ← ↑ ↓ →"
row "Zoom pane"        "Prefix + z"
row "Kill pane"        "Prefix + x"
row "Resize pane"      "Prefix + (hold arrow)"
echo -e "$DIV"
row "─── WINDOWS ─" ""
row "New window"       "Prefix + c"
row "Rename window"    "Prefix + ,"
row "Next / Prev"      "Prefix + n  /  p"
row "List windows"     "Prefix + w"
row "Close window"     "Prefix + &"
echo -e "$DIV"
row "─── SESSION ─" ""
row "Detach"           "Prefix + d  (container lives on)"
row "Reattach"         "./docker.sh  OR"
row ""                 "tmux attach -t dev"
row "Scroll mode"      "Prefix + [  (q to exit)"
row "Command prompt"   "Prefix + :"
echo -e "$BOT"
echo ""

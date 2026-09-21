#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bf_dir="${repo_root}/third_party/betaflight"
patch_dir="${repo_root}/patches/betaflight"
out_dir="${repo_root}/build/betaflight"
worktree="${out_dir}/worktree"

# Legacy FDM semantics: pressure from the packet, quaternion used as sent
bf_extra_flags="-DENABLE_GAZEBO_BRIDGE=0 ${BF_EXTRA_FLAGS:-}"
bf_output_name="${BF_OUTPUT_NAME:-betaflight_SITL.elf}"

if [[ ! -e "${bf_dir}/.git" ]]; then
    echo "error: ${bf_dir} is empty. Run: git submodule update --init third_party/betaflight" >&2
    exit 1
fi

mkdir -p "${out_dir}"
if [[ -d "${worktree}" ]]; then
    git -C "${bf_dir}" worktree remove --force "${worktree}"
fi
git -C "${bf_dir}" worktree prune
git -C "${bf_dir}" worktree add --detach "${worktree}" HEAD >/dev/null

shopt -s nullglob
for patch in "${patch_dir}"/*.patch; do
    echo "applying $(basename "${patch}")"
    git -C "${worktree}" apply --index "${patch}"
done
shopt -u nullglob

# Betaflight's Makefile inits its submodules in parallel jobs that race on the git config lock
while read -r bf_submodule; do
    git -C "${worktree}" submodule update --init -- "${bf_submodule}"
done < <(git -C "${worktree}" config --file .gitmodules --list | awk -F= '
    /^submodule\..*\.path=/   { n = $1; sub(/\.path$/, "", n); paths[n] = $2 }
    /^submodule\..*\.update=/ { n = $1; sub(/\.update$/, "", n); updates[n] = $2 }
    END { for (n in paths) if (updates[n] != "none") print paths[n] }')

make -C "${worktree}" TARGET=SITL EXTRA_FLAGS="${bf_extra_flags}" -j"$(nproc)"

cp "${worktree}/obj/main/betaflight_SITL.elf" "${out_dir}/${bf_output_name}"
git -C "${bf_dir}" describe --tags --always >"${out_dir}/betaflight_version.txt"
echo "built ${out_dir}/${bf_output_name} ($(cat "${out_dir}/betaflight_version.txt"), ${bf_extra_flags})"

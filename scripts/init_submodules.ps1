# Pin the two upstream trees as submodules. This does not copy their source
# into the OrbiEdge directories.
#
# The checklist name avs-simulation/basilisk is not a GitHub organization.
# The Basilisk repository is https://github.com/AVSLab/basilisk

$ErrorActionPreference = "Stop"
Set-Location (Split-Path -Parent $PSScriptRoot)

if (-not (Test-Path ".git")) {
    git init
}

function Add-PinnedSubmodule([string]$Url, [string]$Path, [string]$Branch) {
    if (Test-Path (Join-Path $Path ".git")) {
        Write-Output "$Path is already checked out"
        return
    }
    git submodule add --depth 1 -b $Branch $Url $Path
    if ($LASTEXITCODE -ne 0) {
        throw "git submodule add failed for $Url"
    }
}

Add-PinnedSubmodule "https://github.com/nasa/fprime.git" "third_party/fprime" "devel"
Add-PinnedSubmodule "https://github.com/AVSLab/basilisk.git" "third_party/basilisk" "develop"
git submodule update --init --depth 1

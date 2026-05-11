#!/bin/bash

set -e

GITHUB_USER="jacqueslagnel"
REPO_NAME="lora_watering_master"
REPO_URL="git@github.com:${GITHUB_USER}/${REPO_NAME}.git"

SSH_PRIVATE_KEY="$HOME/.ssh/github.jacqueslagnel"
SSH_PUBLIC_KEY="$HOME/.ssh/github.jacqueslagnel.pub"

# Check SSH key files
if [ ! -f "$SSH_PRIVATE_KEY" ]; then
    echo "ERROR: Private key not found: $SSH_PRIVATE_KEY"
    echo "You have the public key, but Git needs the private key too."
    exit 1
fi

if [ ! -f "$SSH_PUBLIC_KEY" ]; then
    echo "ERROR: Public key not found: $SSH_PUBLIC_KEY"
    exit 1
fi

# Configure SSH for GitHub
mkdir -p "$HOME/.ssh"
chmod 700 "$HOME/.ssh"

if ! grep -q "IdentityFile ~/.ssh/github.jacqueslagnel" "$HOME/.ssh/config" 2>/dev/null; then
    cat >> "$HOME/.ssh/config" <<EOF

Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/github.jacqueslagnel
    IdentitiesOnly yes
EOF
fi

chmod 600 "$HOME/.ssh/config"
chmod 600 "$SSH_PRIVATE_KEY"
chmod 644 "$SSH_PUBLIC_KEY"

# Start SSH agent and add key
eval "$(ssh-agent -s)"
ssh-add "$SSH_PRIVATE_KEY"

# Check GitHub CLI authentication
gh auth status >/dev/null 2>&1 || gh auth login

# Initialize Git repository
git init
git branch -M main

# Commit files
git add .

if git diff --cached --quiet; then
    echo "No files to commit."
else
    git commit -m "Initial commit"
fi

# Create GitHub repository
gh repo create "${GITHUB_USER}/${REPO_NAME}" --private --source=. --remote=origin

# Ensure remote is correct
git remote set-url origin "$REPO_URL"

# Push to GitHub
git push -u origin main


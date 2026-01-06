---
name: huggingface-spaces-deployer
description: This skill helps users deploy their GitHub repositories to Hugging Face Spaces by providing the necessary workflow files, token setup instructions, and deployment procedures.
---

# Hugging Face Spaces Deployer

Deploy GitHub repositories to Hugging Face Spaces with automated GitHub Actions.

## Purpose

This skill provides the necessary files and instructions to deploy a GitHub repository to Hugging Face Spaces, including GitHub Actions workflow configuration and setup instructions.

## When to Use This Skill

Use this skill when users need to:
- Deploy a GitHub repository to Hugging Face Spaces
- Set up automated deployment via GitHub Actions
- Configure Hugging Face token and repository secrets
- Understand the requirements for Hugging Face Spaces deployment

## How to Use This Skill

### 1. Create Hugging Face Token

Guide users to create a Hugging Face token:
1. Navigate to https://huggingface.co/settings/tokens
2. Create a new token with write access
3. Copy the token value (save it securely as it will not be shown again)

### 2. Configure GitHub Repository Secrets

Instruct users to add the token to their GitHub repository:
1. Go to GitHub repository settings: `https://github.com/{username}/{repo}/settings/secrets/actions`
2. Add a new repository secret
3. Name: `HF_TOKEN`
4. Value: the token copied from Hugging Face

### 3. Create GitHub Actions Workflow

Create the workflow file at `.github/workflows/sync_to_huggingface_space.yml` with the following content:

```yaml
name: Sync to Hugging Face hub
on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  sync-to-hub:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
        with:
          fetch-depth: 0
          lfs: true
      - name: Push to hub
        env:
          HF_TOKEN: ${{ secrets.HF_TOKEN }}
        run: git push -f https://$GITHUB_REPOSITORY_OWNER:$HF_TOKEN@huggingface.co/spaces/$GITHUB_REPOSITORY_OWNER/${{ github.event.repository.name }} main
```

### 4. Manual Deployment Option

If users prefer manual deployment instead of GitHub Actions:

1. Add Hugging Face Space as a remote:
   ```bash
   git remote add space https://huggingface.co/spaces/$GITHUB_REPOSITORY_OWNER/${{ github.event.repository.name }}
   ```

2. Set the remote URL with authentication:
   ```bash
   git remote set-url space https://$GITHUB_REPOSITORY_OWNER:$HF_TOKEN@huggingface.co/spaces/$GITHUB_REPOSITORY_OWNER/${{ github.event.repository.name }}
   ```

3. Push to the space:
   ```bash
   git push --force space main
   ```

### 5. Requirements for Hugging Face Spaces

Ensure the repository has the necessary files for Hugging Face Spaces:
- `Dockerfile` at the root level (for backend-only deployment, this should be in the backend folder which becomes the root)
- `requirements.txt` for Python dependencies
- `src/main.py` or appropriate application entry point
- Any other necessary configuration files
- Proper environment variable handling

### 6. Troubleshooting

If deployment fails:
- Verify the HF_TOKEN has write permissions
- Check that the repository name matches the Hugging Face Space name
- Ensure all required files are present in the repository
- Confirm that environment variables are properly configured

#!/usr/bin/env python3
"""
Script to help set up Hugging Face Spaces deployment for a GitHub repository.
"""
import os
import sys
from pathlib import Path

def create_github_workflow(username: str, repo_name: str, token: str = None):
    """
    Create a GitHub Actions workflow file for deploying to Hugging Face Spaces.

    Args:
        username: GitHub/Hugging Face username
        repo_name: Repository name
        token: Hugging Face token (optional, if not provided, will use secrets)
    """
    workflow_content = f"""name: Sync to Hugging Face hub
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

      # Set up a temporary directory with only the backend contents at root
      - name: Prepare backend for Hugging Face Spaces
        run: |
          mkdir -p hf-space-tmp
          cp -r backend/* hf-space-tmp/
          # Ensure the Dockerfile is at the root level for Hugging Face Spaces
          if [ -f "hf-space-tmp/Dockerfile" ]; then
            echo "Dockerfile found in backend, using as root Dockerfile"
          else
            echo "Error: Dockerfile not found in backend folder"
            exit 1
          fi

      # Configure git for Hugging Face
      - name: Configure git for Hugging Face
        run: |
          git config --global user.name "github-actions[bot]"
          git config --global user.email "github-actions[bot]@users.noreply.github.com"

      # Push the backend contents to Hugging Face Spaces
      - name: Push backend to Hugging Face Spaces
        env:
          HF_TOKEN: ${{{{ secrets.HF_TOKEN }}}}
        run: |
          cd hf-space-tmp
          git init
          git remote add origin https://$GITHUB_REPOSITORY_OWNER:$HF_TOKEN@huggingface.co/spaces/$GITHUB_REPOSITORY_OWNER/${{{{ github.event.repository.name }}}}
          git add .
          git commit -m "Update backend for Hugging Face Spaces [skip ci]"
          git push -f origin main
"""

    # Create the .github/workflows directory if it doesn't exist
    workflow_dir = Path(".github/workflows")
    workflow_dir.mkdir(parents=True, exist_ok=True)

    # Write the workflow file
    workflow_file = workflow_dir / "sync_to_huggingface_space.yml"
    with open(workflow_file, 'w') as f:
        f.write(workflow_content)

    print(f"Created GitHub Actions workflow at {workflow_file}")
    return workflow_file

def setup_manual_deployment(username: str, repo_name: str, token: str):
    """
    Provide commands for manual deployment to Hugging Face Spaces.

    Args:
        username: Hugging Face username
        repo_name: Repository name
        token: Hugging Face token
    """
    print("Manual deployment commands:")
    print(f"git remote add space https://huggingface.co/spaces/{username}/{repo_name}")
    print(f"git remote set-url space https://{username}:{token}@huggingface.co/spaces/{username}/{repo_name}")
    print("git push --force space main")

def main():
    if len(sys.argv) < 3:
        print("Usage: python setup_hf_deployment.py <username> <repo_name> [hf_token]")
        print("If HF token is provided, will show manual deployment commands. Otherwise, will create GitHub Actions workflow.")
        sys.exit(1)

    username = sys.argv[1]
    repo_name = sys.argv[2]
    hf_token = sys.argv[3] if len(sys.argv) > 3 else None

    if hf_token:
        setup_manual_deployment(username, repo_name, hf_token)
    else:
        workflow_path = create_github_workflow(username, repo_name)
        print(f"GitHub Actions workflow created at: {workflow_path}")
        print("Remember to add your HF_TOKEN as a GitHub secret named 'HF_TOKEN' in your repository settings.")

if __name__ == "__main__":
    main()
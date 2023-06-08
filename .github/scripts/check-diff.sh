changes() {
  git diff --name-only HEAD $(git merge-base HEAD origin/master)
}

if changes | grep -q $1; then
  echo "CHANGED_$2=${{ steps.check.outputs.should_skip != 'true' }}" >> $GITHUB_OUTPUT
else
  echo "CHANGED_$2=0" >> $GITHUB_OUTPUT
fi

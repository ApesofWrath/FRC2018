#### Helpful Git Commands

# git add -A; git commit -m "Description of changes"
Add all changes to changeset, create commit with changeset including description "Description of changes"

# git log
View commit objects on HEAD

# git fetch --all
Refresh local view of all remote branches

# git log origin/calgames
View commit objects on local branch calgames

# git checkout -b calgamesFeature origin/calgames
Create local branch `calgamesFeature` based on local view of remote branch `calgames`

# git checkout path-to-filename
Revert any unstages changes to file at path-to-filename

# git remote add brett https://github.com/brett668/FRC2018.git
Adds a remote repository named `brett`. You'll want to visit github.com/apesofwrath, fork the FRC2018 repo, and use the repo's clone URL in-place of my fork's URL.

# git branch -M calgamesFeatureBranch
Renames the current local branch to `calgamesFeatureBranch`

# git push brett calgamesFeatureBranch
Pushes the local branch calgamesFeatureBranch to the remote repository brett

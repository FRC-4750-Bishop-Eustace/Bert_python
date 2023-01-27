Open 2022 WPILib VS Code
click Terminal -> New Terminal
type 'git status'
this should output your current branch for git - if this is not your branch, switch to your branch with 'git checkout '(EX git checkout Alex_review)
'git pull' -- this updates your branch with any changes from the github - if any changes need to be committed to your local branch do them now
'git checkout master' -- switches your current branch to master
'git fetch' -- updates all branches from remote repository
'git pull' -- updates master with any changes from the remote repository
'git checkout ' -- switches to your branch
10.'git merge master' this will merge master into your branch

11.'git status' -- if everything is up to date this will let you know, if there are more issues reach our to Mr. Carlin or Mr. Monari

Git Commands

Pull request - click "new pull requests"
Select base branch ; select compare
Click "create pull requests"
Create a title and a comment describing what you are doing (EX: title = merge master into my branch; comment = updating my branch with master);
Click "create pull requests"
If there are merge conficts, select "resolve conflicts"; Then in order to resolve conflicts choose the version that you would like to use and delete the spacers indicating a merge (======= or >>>>>>> or <<<<<<<)
WIP URLs

https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
-Mr.Monari

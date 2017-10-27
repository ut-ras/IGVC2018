# Contributing Guide

> Chaos is inherent in all compound things. Strive on with diligence.

> -Buddah

If we want to do anything complex without it all crashing and burning, we need to be disciplined in how we develop our code. This guide will assume some familiarity with git, if anything is over your head just ask on the slack.

## Adding and changing code
If you want to develop some feature of the code base there is one thing you need to remember above all else: **MAKE BRANCHES**. Messing with the IMU drivers? checkout an IMU branch. Editing this very file? checkout a housekeeping branch. Nothing that you write should ever be done directly on top of master. In fact, we have protected the master branch so that even if you have write permissions for the repo, you cannot push to master. This may sound like it's making your life unduly difficult for no reason, but months or years down the line, it's the only way to keep the history intelligible. Make sure to keep other branches up to date with `git fetch` and `git rebase origin/master`. When you're done with your code, test it, document it, get someone else to look at it, document it some more, and _then_ open a PR with a full description of the changes.

## Documentation
The rules are simple: if you don't write docstrings, your code doesn't get merged. Unless a function is obscenely simple, it probably needs a docstring. Commit messages are also documentation. They should be descriptive and to the point.

## Project management
If you find a bug in the code, make an issue. If you aren't sure what needs to be worked on, check the issues. If you have an idea for a feature we should add, make an issue. Tag issues appropriately and make them readable.

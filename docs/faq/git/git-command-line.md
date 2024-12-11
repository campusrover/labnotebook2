---
title: Why can't I modify git from command line when I have permission?
author: Zared Cohen
date: dec-10-2024
status: new
---

## Background

Ever wonder why sometimes when you try to make changes to a git you KNOW you have permissions on doesn't work via command line? Whenever you try, this error occurs:

`remote: Permission to <git name>.git denied to <username>.`

Well, fear not. I have the perfect solution for you, so you don't have to worry about logging out and logging in again over and over to try and fix the issue! The answer? Personal Access Tokens! With these, you can make changes to your git from anywhere with just one line! Below are the steps needed to do so:

### Getting your Personal Access Token

First, you need to copy an active Personal Access Token from git, or if you don't have one, you can create one. 

1. Go to github.com, and make sure you are logged in
1. Click your profile photo in the top right, and click settings in the drop down mene
1. Scroll down to the bottom of the menu on your left and click on **Developer Settings**
1. Click the Personal access tokens drop down menu, and click **Tokens (classic)**
1. On the top right, click **Generate new token**
1. Give your new token a name, set the expiration to as long as you will be working on your project, and make sure all of the checkboxes underneath **repo** are checked
1. Click **Generate token**

Voila! You have created a fresh new token. Copy this and you can use it in your terminal or command line.

### Using your token

Now that you have your token go into a terminal or command line of your choice, and navigate to your git directory using the `cd` command.

Once there, run the following line:

`git remote set-url origin https://<username>:<token>@github.com/<your github repository name>.git`

And fill in the <> as needed. And thats it. Now just push your changes and your done!

`git add .`
`git commit -m "message"`
`git push`


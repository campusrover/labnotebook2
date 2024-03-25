---
title: FAQ
author: Pito Salas
description: All sorts of small topics are covered here
---
# FAQ

## Steps for contributing to this section

1. Git clone this repository to your computer: [campusrover/labnotebook](https://github.com/campusrover/labnotebook)
2. In the directory structure, find the folder called `faq`. In it you can create a `.md` file for your contribution.
3. Write your contribution using markdown. You can include images as well as links. There are many tutorials on Markdown but you most likely already know it.
4. If you want the title to be something other than the file name, then add:
```
---
title: Gooder Title
author: Pito Salas
description: nicer subtitle
order: where does this one appear in order
status: obsolete|new|tophit
date: month-year
---
```
as the first few lines of the file
5. Git add, commit and push your changes.

## Note
Brand new files (vs. edits) will not appear in the directory until it is rebuilt by rpsalas@brandeis.edu so send them an email!

# Guide to contributing to the Lab Notebook
*Author: Pito Salas, Date: April 2023*

When contributing, please make sure to do the following:

1. Proofread your writing for spelling and grammatical errors, especially in areas where excat typing is important (e.g. code snippets)
2. Place your files in the appropriate folders. Only create new folders if absolutely necessary. If you make a new folder, give it a README which clearly describes the content of the folder
3. Add your file to Summary.md so it is accessible from the gitbook webpage
4. Give your images common sense names. They should be descriptive and/ or associative. "p1.png" is bad, "gazebo_img1.png" is better.
5. Only use hyphens, not underscores, in filenames.
6. Try to order files in SUMMARY.md is a way that makes sense if they were to be read through sequentially
7. Follow Markdown format guidleines. There is a Markdown extenstion for VS code which will assist you in this endeavor. Here are some common mistakes:

   1. No trailing spaces at the end of lines
   2. One (and only one) blank new line on either side of Headers, lists, and code blocks
   3. Use consistant spacing
   4. no embedded HTML or raw links
   5. no trailing punctuation in headers
   6. Do not skip header levels, only one H1

8. Furthermore, please follow these style guidelines:

   1. Do not use bold or italics in headers
   2. Do not use emphasis to take the place of a code snippet
   3. Label your code snippets \(starting with \`\`\`)with the language they contain! common languages in this course are `python`, `bash`, `xml` and `c++`
   4. Surround package names, file paths and other important info of this nature with single backticks \(\`)
   5. Add a byline at the bottom of the page, including your name and date of publishing


# Contributing Guide

## Getting started

Information on how we write our markdown files can be found in [the GitHub Markdown reference](contributing/content-markup-reference.md).

### Issues

#### Create a new issue

If you spot a problem with the docs, [search if an issue already exists](https://docs.github.com/en/github/searching-for-information-on-github/searching-on-github/searching-issues-and-pull-requests#search-by-the-title-body-or-comments). If a related issue doesn't exist, you can open a new issue using a relevant [issue form](https://github.com/dnovischi/jetson-tutorials/issues/new/choose).

#### Solve an issue

Scan through our [existing issues](https://github.com/dnovischi/jetson-tutorials/issues) to find one that interests you. You can narrow down the search using `labels` as filters. See [Labels](/contributing/how-to-use-labels.md) for more information. As a general rule, we don’t assign issues to anyone. If you find an issue to work on, you are welcome to open a PR with a fix.

### Make Changes

#### Make changes locally

1. Fork the repository.
- Using the command line:
  - [Fork the repo](https://github.com/dnovischi/jetson-tutorials/fork) so that you can make your changes without affecting the original project until you're ready to merge them.
  - Clone your own fork locally. If your run into problems durring this step, read the [Set up Git](https://docs.github.com/en/get-started/quickstart/set-up-git) page from GitHub's documentation.
  - Navigate to your local repository"
  ```bash
  cd jetson-tutorials
  ```
  - Checkout your fork is the `origin` remote
  ```bash
  git remote add origin URL_OF_FORK
  ```
  - Add the project repository as the `upstream` remote:
  ```bash
  git remote add upstream https://github.com/dnovischi/jetson-tutorials.git
  ```
  - Pull the latest changes from upstream main and development branches into your local repository:
  ```bash
  git pull upstream main
  git pull upstream development
  ```

2. Create a working branch and start with your changes!
```bash
git checkout -b BRANCH_NAME
```

### Commit your update

1. Commit the changes once you are happy with them:
```bash
git add -A # Stage the changes
git commit -m DESCRIPTION OF CHANGES # Commit changes
```
2. Push your changes to your fork:
```bash
git push origin BRANCH_NAME
```


### Pull Request

When you're finished with the changes, create a pull request pointed towards the `development` branch, also known as a PR:

1. To begin the pull request, return to your fork on GitHub, and refresh the page. You may see a highlighted area that displays your recently pushed branch.
2. Click the green Compare & pull request button to begin the pull request.
3. Create the pull request towards the `development` branch.
4. Before submitting the pull request, you first need to describe the changes you made (rather than asking the project maintainers to figure them out on their own). You should write a descriptive title for your pull request, and then include more details in the body of the pull request. If there are any related GitHub issues, make sure to mention those by number. The body can include Markdown formatting, and you can click the Preview tab to see how it will look.
5. If everything looks good, click the green Create pull request button!
6. You can continue to add more commits to your pull request even after opening it! For example, the project maintainers may ask you to make some changes, or you may just think of a change that you forgot to include:
  - start by returning to your local repository, and use `git branch` to see which branch is currently checked out. If you are currently in the `development` branch (rather than the branch you created), use `git checkout BRANCH_NAME` to switch.
  - Then, you should repeat steps to make changes, commit them, and push them to your fork.

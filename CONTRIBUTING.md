# How to contribute

**Any contributions to extend the functionality or to solve existing problems are welcome!**

Please read this document before contributing to this project. It explains the preferred way of how to contribute to this project.

# Maintaining & Writing Code

When writing code, make sure it is consistent with the project's naming conventions and that it is well tested. Please read and follow the [GitHub Flow](https://docs.github.com/en/get-started/using-github/github-flow) guidelines when contributing to this project.

This project uses [pre-commit](https://pre-commit.com) to run various checks on the code before committing. So, before making any commits, please install pre-commit and the pre-commit hooks by running the following commands:

```bash
# Install pre-commit.
pip install pre-commit

# Install the hooks.
pre-commit install
```

See [the official documentation](https://pre-commit.com/#install) for more information on how to install pre-commit.

## Creating an Issue

Before creating a new issue on GitHub, please...

1. Make sure there is no open issue on GitHub for the same topic yet.
2. Use the respective issue template for the type of issue you are creating (e.g. bug report, feature request, etc.).
3. Fill out the issue template completely and provide all necessary information, if using a template.

## Submitting a Pull Request

1. Push your changes to your topic branch on your fork of the repo.
2. Submit a pull request from your topic branch to the main branch on this project's repository.
3. Be sure to tag any issues your pull request is contributing to (i.e. `Fixes #123`). _This will automatically close the issue once the pull request is merged._

## Merging a Pull Request - _for Maintainers_

Before merging a pull request, make sure of the following.

1. The pull request has been approved by the reviewer(s).
2. The pull request has been reviewed by at least one other person.
3. The pull request has been tested and all tests are passing.

When merging a pull request, use the "Squash and Merge" option to keep the commit history clean and concise. Make sure to write a clear commit message that describes what functionality is added or changed.

## Testing

If you want to submit a bug fix or new feature, make sure that all tests are passing. If you are adding new functionality, please also add tests for it or update the existing tests, if necessary. Examples (in the [examples](examples) directory) can also serve as tests, so make sure they are working as expected.

# Course Page: Introduction to Practical Robotics 

## Setup tools

* Install required packages

```
$ sudo apt-get install ruby-full build-essential zlib1g-dev
```

Add the following content to your "~/.bashrc"

```
# Install Ruby Gems to ~/gems'
export GEM_HOME="$HOME/gems"
export PATH="$HOME/gems/bin:$PATH"
```

Do not forget to source the ~/.bashrc" after change.

```
$ gem install jekyll bundler github-pages
```

## Add Contents

First clone the repository to your computer:

```
$ git clone https://github.com/westonrobot/robotics101
```

Then you can modify the files and preview the site:

```
$ bundle exec jekyll serve --livereload
```

Open the preview page at: http://localhost:4000/robotics101/

## Reference

* https://jekyllrb.com/docs/
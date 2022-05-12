---
layout: home
title: Home
nav_exclude: false
permalink: index.html
seo:
  type: Course
  name: Introduction to Practical Robotics
nav_order: 1
---

# Robotics 101

Welcome to the **Introduction to Practical Robotics** class! 

You can find useful information about the course here:

- [announcements](announcements.md)
- [weekly schedule](schedule.md)
- [online resources](resources.md)

It's highly recommended that you go through the syllabus and course schedule at the beginning of the course. This will help you get an idea about what you can expect from the course so that you can better plan your study during this semester.

## Course Staff

### Instructor

{% assign instructors = site.staffers | where: 'role', 'Instructor' %}
{% for staffer in instructors %}
{{ staffer }}
{% endfor %}

{% assign teaching_assistants = site.staffers | where: 'role', 'Teaching Assistant' %}
{% assign num_teaching_assistants = teaching_assistants | size %}
{% if num_teaching_assistants != 0 %}

### Teaching Assistants

{% for staffer in teaching_assistants %}
{{ staffer }}
{% endfor %}
{% endif %}

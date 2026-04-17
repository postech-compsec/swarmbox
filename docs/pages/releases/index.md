---
layout: default
title: Release Notes
nav_order: 98
has_children: true
permalink: /releases/
has_toc: false
---


# Release Notes
This section provides detailed release notes for each version of SwarmBox. 
Each release note pages are grouped by their minor version (e.g., v0.1.x) to help users easily track the evolution of features and bug fixes within each major release.

## All Versions

{% for page in site.pages %}
  {% if page.parent == "Release Notes" %}
* [{{ page.title }}]({{ page.url | relative_url }})
  {% endif %}
{% endfor %}
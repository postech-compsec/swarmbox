---
layout: default
title: Release Notes
nav_order: 4
permalink: /releases/
has_toc: false
---


# Release Notes
This page shows the latest SwarmBox release note.  
For older releases, check the [GitHub Releases](https://github.com/postech-compsec/swarmbox/releases) page.

{% assign latest_release = site.github.latest_release | default: site.github.releases.first %}
{% assign releases_url = site.github.repository_url | append: "/releases" %}

{% if latest_release %}
<!-- ## Latest Release -->

# {{ latest_release.name | default: latest_release.tag_name }}

> **Tag:** {{ latest_release.tag_name }}, **Date:** {{ latest_release.published_at | date: "%Y-%m-%d" }}  
> [View on GitHub]({{ latest_release.html_url }})  
> [All Releases]({{ releases_url }})
{: .release }

{{ latest_release.body | markdownify }}
{% else %}
No published releases were found.

[View GitHub Releases]({{ releases_url }})
{% endif %}
"""Building a component cache from scratch: the weld_cache session's preparation sequence as importable steps.

Each step module has ``run(layout, ...)`` with the session's defaults; :mod:`.pipeline` runs them in the order of
docs/umarell_jam.md ("Prepare"). Every step reads and publishes checksummed components in ``layout.components``
(``Store.obtain``: certified entries are reused, timeouts are retried), so an interrupted run resumes.
"""

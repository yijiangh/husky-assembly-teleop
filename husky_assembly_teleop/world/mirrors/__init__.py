"""
Mirrors: private copies of a `SceneSnapshot` in one collision backend each, for planners.

! A mirror belongs to one thread, usually a planner's worker. Backend ids never leave it.
"""

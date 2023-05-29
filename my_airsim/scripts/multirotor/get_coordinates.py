import unreal

actorsList = unreal.EditorLevelLibrary.get_all_level_actors()
for actor in actorsList:
    actorLabel = actor.get_actor_label()
    actorPos = actor.get_actor_location()
    if (actorLabel == 'cylinder_bp'):
        print('actorLabel= %s actorPos=%s' % (actorLabel, actorPos))
def itj_module_folder():
    import os
    import integral_timber_joints
    return os.path.realpath(os.path.join(os.path.dirname(integral_timber_joints.__file__), '..', '..'))


def itj_submodule_folder():
    import os
    return os.path.realpath(os.path.join(itj_module_folder(), 'external'))


if __name__ == "__main__":
    print(itj_module_folder())
    print(itj_submodule_folder())

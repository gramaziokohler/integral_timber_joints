import rhinoscriptsyntax as rs
import compas
import compas_fab
import os
import compas_bootstrapper

def print_package_info():
    compas_version  = compas.__version__
    compas_path = os.path.abspath(compas.__file__)

    compas_fab_version = compas_fab.__version__
    compas_fab_path = os.path.abspath(compas_fab.__file__)

    conda_env_name = compas_bootstrapper.ENVIRONMENT_NAME
    python_directory = compas_bootstrapper.PYTHON_DIRECTORY

    print ('compas_version = %s' % compas_version)
    print ('compas_path = %s' % compas_path)
    print ('compas_fab_version = %s' % compas_fab_version)
    print ('compas_fab_path = %s' % compas_fab_path)
    print ('conda_env_name = %s' % conda_env_name)
    print ('python_directory = %s' % python_directory)
    return (
        compas_version, compas_path,
        compas_fab_version, compas_fab_path,
        conda_env_name, python_directory,        
        )


if __name__ == "__main__":
    print_package_info()
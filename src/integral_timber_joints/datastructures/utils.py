# This file originates from Keerthana Udaykumar's <kudaykum@student.ethz.ch> master thesis on `Timber Grammar` in Fall 2019
# Pulled from https://github.com/KEERTHANAUDAY/timber_grammar on 2019-10-09 commit: d389364304f3740e652dbe4e85291402a2df0636
# This file is there on modified by Victor Leung <leung@arch.eth.ch> for use in Integral Timber Joint project

# import System
import uuid

def create_id():
    """Generates a UUID
    Return:
    ------
    UUID
    """
    g = str(uuid.uuid1())
    return g

if __name__ == "__main__":
    a = create_id()
    print(type(a))
    print(a)

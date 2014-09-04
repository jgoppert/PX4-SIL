import re
import sympy


class SympyDict(dict):
    """
    This saves a dictonary of sympy expressions to a file
    in human readable form.
    >>> a, b = sympy.symbols('a, b')
    >>> d = SympyDict({'a':a, 'b':b})
    >>> d.save('name.sympy')
    >>> del d
    >>> d2 = SympyDictd.load('name.sympy')
    """

    def __init__(self, *args, **kwargs):
        super(SympyDict, self).__init__(*args, **kwargs)

    def __repr__(self):
        d = dict(self)
        for key in d.keys():
            d[key] = sympy.srepr(d[key])
        # regex is just used here to insert a new line after
        # each dict key, value pair to make it more readable
        return re.sub('(: \"[^"]*\",)', r'\1\n',  d.__repr__())

    def save(self, file):
        with open(file, 'wb') as savefile:
            savefile.write(self.__repr__())

    @classmethod
    def load(cls, file_path):
        with open(file_path, 'r') as loadfile:
            exec('d =' + loadfile.read())
        d = locals()['d']
        for key in d.keys():
            d[key] = sympy.sympify(d[key])
        return cls(d)


def state_space_lambdify(f_vect, x_vect, u_vect, *args):
    """
    Given a state space description of a dynamic system, create
    a lambda funtion to simulate it with.

    Input
        f_vect : sympy matrix of expressions for
            the derivative of x (continuous),
            or for the change in x (discrete)
        x_vect : sympy matrix of symbols in x vector
        u_vect : sympy matrix of symbols in u vector
        *args: other parameters that are not states or inputs
    """
    x = sympy.DeferredVector('x')
    u = sympy.DeferredVector('u')
    ss_subs = {x_vect[i]: x[i] for i in range(len(x_vect))}
    ss_subs.update({u_vect[i]: u[i] for i in range(len(u_vect))})
    return sympy.lambdify((x, u) + args, f_vect.subs(ss_subs))

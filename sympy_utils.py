import sympy
from numpy import f2py
import pprint
from sympy.utilities import codegen
import StringIO


def rhs_to_scipy_ode(rhs, t, x_vect, u_vect,
                     constants, *args, **lambdify_kwargs):
    """Convert rhs to lambda function and jacobian comaptible with scipy.

    Given a state space description of a dynamic system, create
    a lambda function with a signature compatible with
    scipy.ode.integrate: f(t, x, u, *args).

    Parameters
    ----------
    rhs : sympy.Matrix
        Column vector of expressions for
        the derivative of x (continuous),
        or for the change in x (discrete).
    t : sympy.Symbol
        The independent time-like variable.
    x_vect : sympy.Matrix
        Column vector of symbols in the x (state) vector.
    u_vect : sympy.Matrix
        Column vector of symbols in the u (input) vector.
        Most controllers pass a vector of inputs and this
        supports this structure.
    constants: dict
        Dictionary of constants to substitute into rhs.
    *args: dict
        Additioinal symbols in f_vect that are not states, inputs
        or constants.
    **lambdify_kwargs:
        Additional arugments to pass to lambdify.
        @see sympy.utilities.lambdify.lambdify

    Returns
    -------
    f : function
        A lambda function that compute the right hand side
        of the ode: f(t, x, u, *f_args).
    jac : function
        A lambda function that computes the jacobian
        of the right hand side with respect to the state x:
        jac(t, x, u, *jac_args).
    """
    if constants is not None:
        rhs = rhs.subs(constants)
    x = sympy.DeferredVector('x')
    u = sympy.DeferredVector('u')
    ss_subs = {x_vect[i]: x[i] for i in range(len(x_vect))}
    ss_subs.update({u_vect[i]: u[i] for i in range(len(u_vect))})
    if 'default_array' not in lambdify_kwargs.keys():
        lambdify_kwargs['default_array'] = True
    f = sympy.lambdify((t, x, u) + args, rhs.subs(ss_subs),
                       **lambdify_kwargs)
    jac_vect = rhs.jacobian(x_vect)
    jac = sympy.lambdify(
        (t, x, u) + args, jac_vect.subs(ss_subs),
        **lambdify_kwargs)
    return (f, jac)


def save_sympy_expr(expr, filename):
    with open(filename, 'wb') as save_file:
        save_file.write(sympy.python(expr))


def load_sympy_expr(filename):
    with open(filename, 'r') as load_file:
        load_string = load_file.read()
    exec('from sympy import *')
    exec(load_string)
    e = locals()['e']
    return e


def save_repr(d, filename, env_string=''):
    with open(filename, 'wb') as save_file:
        save_file.write(env_string)
        save_file.write('e=' + pprint.saferepr(d))


def load_repr(filename):
    with open(filename, 'r') as load_file:
        load_string = load_file.read()
    exec(load_string)
    e = locals()['e']
    return e


def gen_fortran_module(module, routines, project, header=True):
    fgen = codegen.FCodeGen(project)
    s = StringIO.StringIO()
    fgen.dump_f95(routines, s, module, header=header)
    src = s.getvalue()
    s.close()
    return src


def compile_fortran_module(src, module):
    if f2py.compile(src, 'pendulum') != 0:
        raise RuntimeError('f2py compilation failed:\n' + src)

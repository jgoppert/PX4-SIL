!******************************************************************************
!*                    Code generated with sympy 0.7.5-git                     *
!*                                                                            *
!*              See http://www.sympy.org/ for more information.               *
!*                                                                            *
!*                       This file is part of 'PX4-SIL'                       *
!******************************************************************************

subroutine f(x_dot_, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:2, 1:1) :: x_dot_
REAL*8, intent(in), dimension(1:2, 1:1) :: x

x_dot_(1, 1) = x(2, 1)
x_dot_(2, 1) = (g*l*m*sin(x(1, 1)) + u)/(l**2*m)

end subroutine

subroutine f_lin(A_, B_, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:2, 1:2) :: A_
REAL*8, intent(out), dimension(1:2, 1:1) :: B_
REAL*8, intent(in), dimension(1:2, 1:1) :: x

A_(1, 1) = 0
A_(2, 1) = g*cos(x(1, 1))/l
A_(1, 2) = 1
A_(2, 2) = 0
B_(1, 1) = 0
B_(2, 1) = 1/(l**2*m)

end subroutine

subroutine y(y_, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:3, 1:1) :: y_
REAL*8, intent(in), dimension(1:2, 1:1) :: x

y_(1, 1) = -u/(l*m)
y_(2, 1) = 0
y_(3, 1) = -g*cos(x(1, 1)) + l*x(2, 1)**2

end subroutine

subroutine C(C_, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:3, 1:2) :: C_
REAL*8, intent(in), dimension(1:2, 1:1) :: x

C_(1, 1) = 0
C_(2, 1) = 0
C_(3, 1) = g*sin(x(1, 1))
C_(1, 2) = 0
C_(2, 2) = 0
C_(3, 2) = 2*l*x(2, 1)

end subroutine

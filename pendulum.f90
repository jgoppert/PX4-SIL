!******************************************************************************
!*                    Code generated with sympy 0.7.5-git                     *
!*                                                                            *
!*              See http://www.sympy.org/ for more information.               *
!*                                                                            *
!*                       This file is part of 'PX4-SIL'                       *
!******************************************************************************

subroutine compute_f(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:2, 1:1) :: out
REAL*8, intent(in), dimension(1:2, 1:1) :: x

out(1, 1) = x(2, 1)
out(2, 1) = g*sin(x(1, 1))/l + u/(l**2*m)

end subroutine

subroutine compute_A(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:2, 1:2) :: out
REAL*8, intent(in), dimension(1:2, 1:1) :: x

out(1, 1) = 0
out(2, 1) = g*cos(x(1, 1))/l
out(1, 2) = 1
out(2, 2) = 0

end subroutine

subroutine compute_B(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:2, 1:2) :: out
REAL*8, intent(in), dimension(1:2, 1:1) :: x

out(1, 1) = 0
out(2, 1) = g*cos(x(1, 1))/l
out(1, 2) = 1
out(2, 2) = 0

end subroutine

subroutine compute_g_accel(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:3, 1:1) :: out
REAL*8, intent(in), dimension(1:2, 1:1) :: x

out(1, 1) = -u/(l*m)
out(2, 1) = 0
out(3, 1) = -g*cos(x(1, 1)) + l*x(2, 1)**2

end subroutine

subroutine compute_g_accel_H(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:3, 1:2) :: out
REAL*8, intent(in), dimension(1:2, 1:1) :: x

out(1, 1) = 0
out(2, 1) = 0
out(3, 1) = g*sin(x(1, 1))
out(1, 2) = 0
out(2, 2) = 0
out(3, 2) = 2*l*x(2, 1)

end subroutine

subroutine compute_g_gyro(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:3, 1:1) :: out
REAL*8, intent(in), dimension(1:2, 1:1) :: x

out(1, 1) = 0
out(2, 1) = x(2, 1)
out(3, 1) = 0

end subroutine

subroutine compute_g_gyro_H(out, t, x, u, m, g, l)
implicit none
REAL*8, intent(in) :: t
REAL*8, intent(in) :: x
REAL*8, intent(in) :: u
REAL*8, intent(in) :: m
REAL*8, intent(in) :: g
REAL*8, intent(in) :: l
REAL*8, intent(out), dimension(1:3, 1:2) :: out

out(1, 1) = 0
out(2, 1) = 0
out(3, 1) = 0
out(1, 2) = 0
out(2, 2) = 1
out(3, 2) = 0

end subroutine

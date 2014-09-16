import collections
import numpy as np
import matplotlib.pyplot as plt


class TunnelRecord(
    collections.namedtuple(
        'TunnelRecord',
        ['time_stamp', 'temp_C', 'dynamic_pressure_Pa',
         'velocity_m_per_sec', 'drag_N', 'lift_N',
         'side_N', 'pitch_N_m', 'roll_N_m', 'yaw_N_m',
         'aoa_rad'])):

    @classmethod
    def load(cls, path, skip=[]):
        # conversion factors
        in_h20_to_Pa = 249
        lbs_to_N = 4.44822162825
        ft_lbs_to_N_m = 1.35581795

        # read file
        with open(path) as f:
            lines = f.readlines()
        new_lines = []
        for i in range(len(lines)):
            if i in skip:
                continue
            new_lines.append(lines[i].strip().split('\t'))
        new_lines = np.array(new_lines).T
        time_stamp = new_lines[0, :]
        data = new_lines[1:, :].astype(float)

        # convert and create data structure
        return cls(
            time_stamp=time_stamp,
            temp_C=data[0, :],
            dynamic_pressure_Pa=data[1, :]*in_h20_to_Pa,
            velocity_m_per_sec=data[2, :],
            drag_N=data[3, :]*lbs_to_N,
            lift_N=-data[4, :]*lbs_to_N,
            side_N=data[5, :]*lbs_to_N,
            roll_N_m=data[6, :]*ft_lbs_to_N_m,
            pitch_N_m=data[7, :]*ft_lbs_to_N_m,
            yaw_N_m=data[8, :]*ft_lbs_to_N_m,
            aoa_rad=np.deg2rad(data[9, :]))


class AeroCoefficients(
    collections.namedtuple(
        'AeroCoefficients',
        ['aoa_rad', 'S', 'c_bar', 'q_bar',
         'CL', 'CD', 'CY',
         'Cl', 'Cm', 'Cn'])):

    @classmethod
    def from_TunnelRecord(cls, tr, S, c_bar):
        CL = tr.lift_N/tr.dynamic_pressure_Pa/S
        CD = tr.drag_N/tr.dynamic_pressure_Pa/S
        CY = tr.side_N/tr.dynamic_pressure_Pa/S
        # TODO proper non-dim for moments
        Cl = tr.roll_N_m/tr.dynamic_pressure_Pa/S/c_bar
        Cm = tr.pitch_N_m/tr.dynamic_pressure_Pa/S/c_bar
        Cn = tr.yaw_N_m/tr.dynamic_pressure_Pa/S/c_bar
        aoa_rad = tr.aoa_rad
        q_bar = tr.dynamic_pressure_Pa.mean()
        return cls(aoa_rad=aoa_rad, S=S,
                   c_bar=c_bar, q_bar=q_bar,
                   CL=CL, CD=CD, CY=CY,
                   Cl=Cl, Cm=Cm, Cn=Cn)

    def subtract(self, mounts_ac):
        return AeroCoefficients(
            aoa_rad=self.aoa_rad, S=self.S, c_bar=self.c_bar,
            CL=self.CL - mounts_ac.CL.mean(),
            CD=self.CD - mounts_ac.CD.mean(),
            CY=self.CY - mounts_ac.CY.mean(),
            Cl=self.Cl - mounts_ac.Cl.mean(),
            Cm=self.Cm - mounts_ac.Cm.mean(),
            Cn=self.Cn - mounts_ac.Cn.mean(),
            q_bar=self.q_bar)

    def fit(self):
        k_CD_CL, b, c = np.polyfit(self.CL, self.CD, 2)
        CL_DM = -b/(2*k_CD_CL)
        CD_0 = c - k_CD_CL*CL_DM**2
        CL_alpha, CL_0 = np.polyfit(self.aoa_rad, self.CL, 1)
        Cl_alpha = np.polyfit(self.aoa_rad, self.Cl, 1)[0]
        Cm_alpha = np.polyfit(self.aoa_rad, self.Cm, 1)[0]
        Cn_alpha = np.polyfit(self.aoa_rad, self.Cn, 1)[0]
        return {
            'C_L_DM': CL_DM,
            'C_D_0': CD_0,
            'k_CD_CL': k_CD_CL,
            'C_L_0': CL_0,
            'C_L_alpha': CL_alpha,
            'C_l_alpha': Cl_alpha,
            'C_m_alpha': Cm_alpha,
            'C_n_alpha': Cn_alpha}

    def plot(self):
        aoa_deg = np.rad2deg(self.aoa_rad)

        plt.figure(figsize=(15, 5))

        plt.subplot(131)
        plt.plot(aoa_deg, self.CL, '.')
        plt.xlabel(r'$\alpha$, deg')
        plt.ylabel('CL')
        plt.grid()

        plt.subplot(132)
        plt.plot(self.CL, self.CD, '.')
        plt.xlabel('CL')
        plt.ylabel('CD')
        plt.grid()

        plt.subplot(133)
        h = []
        h.append(plt.plot(aoa_deg, self.Cl, '.')[0])
        h.append(plt.plot(aoa_deg, self.Cm, '.')[0])
        h.append(plt.plot(aoa_deg, self.Cn, '.')[0])
        plt.legend(h, ['Cl', 'Cm', 'Cn'], loc='best')
        plt.xlabel(r'$\alpha$, deg')
        plt.ylabel('Cl, Cm, Cn')
        plt.grid()

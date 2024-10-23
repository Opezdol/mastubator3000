from dataclasses import dataclass

from build123d import *  # type:ignore

## define GLOBALS of the model
BLADE_N4_PTS = [(0, 0), (75, -35), (145, 10)]
D_BLADE_PTS = [
    (0, 0),
    (75, -15),
]


@dataclass
class Base_Blade:
    wire: Spline
    width = 25
    blade_thick = 18
    cam_r = 8
    cam_l = 44.7
    cam_offset = 1.5  # both sides
    r = 2

    # calc props ZONE
    @property
    def blade_lip_width(self):
        return self.cam_r + self.cam_offset * 2

    ## We build sketches for bodies

    def sk_v(self, scalar: float = 1) -> Sketch:
        cal_blade_thick = self.blade_thick * scalar

        mv = -(self.width / 2) + self.blade_lip_width / 2
        res = Pos(X=-mv) * Rectangle(
            self.blade_lip_width,
            cal_blade_thick,
            align=(Align.CENTER, Align.MAX, Align.CENTER),  # type:ignore
        )
        return res

    def sk_h(
        self, scalar: float = 1 , r: float = 1, w_scale: float = 1
    ) -> Sketch:

        cal_blade_thick = self.blade_thick * scalar
        res = RectangleRounded(self.width * w_scale, cal_blade_thick, r,
            align=(Align.CENTER, Align.MAX, Align.CENTER))  # type:ignore
        return res



    def cam(self):
        pos_v = self.wire @ 0.8
        rot_v = self.wire % 0.7
        wha = self.wire ^ 0.7
        print(f"_____\nPos \t{pos_v}\nRot{rot_v}______\n")
        x_offset = self.width / 2 - self.blade_lip_width / 2
        loc = Location(pos_v, rot_v)
        sk = Pos(Y=-x_offset) * Circle(5.5 / 2)
        res = (self.wire ^ 0.7) * sk
        # res = loc * sk
        return extrude(res, amount=40)

    def sk_init(self)->Sketch:
        hor_sketch = self.sk_h(scalar=2,r= 4)
        return hor_sketch


    def sk_body(self, scale_v, scale_h, inner_fillet = 2, outer_fillet = 2):
        hor_sketch = self.sk_h(scale_h)
        ver_sketch = self.sk_v(scale_v)
        ## Verts manipulation
        vert_snapshot = hor_sketch.vertices() + ver_sketch.vertices()
        base_no_fillet = hor_sketch + ver_sketch
        new_verts = base_no_fillet.clean().vertices() - vert_snapshot
        v_inner = new_verts.sort_by(Axis.Y)[1]
        ## sketchz zone
        sk_inner_fillet = fillet(v_inner, inner_fillet)
        sk_outer_fillet = fillet(base_no_fillet.vertices() - v_inner, outer_fillet)

        show_object(sk_outer_fillet & sk_inner_fillet, name="sk_outer_fillet")
        show_object(sk_inner_fillet & sk_outer_fillet, name="sk_inner_fillet")
        return base_no_fillet

    def blade_h(self):
        """
        combines 2 base sketches
        """
        show_object(self.wire, name='wire')
        rrt = Rot(Z=-90)
        body = [
            (self.wire ^ 0)  *rrt * self.sk_init(),
            (self.wire ^ 0.25) * rrt * self.sk_h(0.5),
            (self.wire ^ 0.85) *rrt *  self.sk_h(0.3),
            #(self.wire ^ 0.95) *rrt *  self.sk_h(0.2),
            (self.wire ^ 1) *rrt *  self.sk_h(0.15, w_scale=0.69),
        ]
        res = loft(body)
        return res

    def build(self):
        """
        Build the blade by defining different 2D "Sketch" positions on given curve aka his majesty-"self.wire"
        """
        # dev zone
        return self.sk_body(scale_v=1, scale_h=0.8)


blade_spline = Spline(BLADE_N4_PTS)
test = Base_Blade(wire=blade_spline)
show_object(test.sk_init(), name="sk_init")
show_object(test.blade_h(), name="pre_sk_all")

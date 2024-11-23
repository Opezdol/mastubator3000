from dataclasses import dataclass

from build123d import *  # type:ignore

## define GLOBALS of the model
# BLADE_N4_PTS = [(5, 0), (75, -35), (145, 10)]  # archv

BLADE_N4_PTS = [(5, 0), (110, -30), (145, 10)]
D_BLADE_PTS = [
    (0, 0),
    (75, -15),
]


@dataclass
class Base_Blade:
    wire: Spline
    width = 25
    blade_thick = 14
    cam_r = 8
    cam_l = 44.7
    cam_offset = 1.5  # both sides
    cam_wire_r = 3.65 / 2
    r = 2
    hand_w = 45

    # calc props ZONE
    @property
    def blade_lip_width(self):
        return self.cam_r + self.cam_offset * 2

    ## We build sketches for bodies

    def sk_v(
        self,
        scalar: float = 1,
        r: float = 2,
        vertical_offst: float = 3,
        hor_offst: float = 0,
    ) -> Sketch:
        cal_blade_thick = self.blade_thick * scalar

        mv = -(self.width / 3) + self.blade_lip_width / 2

        sk = RectangleRounded(
            self.blade_lip_width,
            cal_blade_thick + vertical_offst,
            r,
            align=(Align.CENTER, Align.MIN, Align.CENTER),  # type:ignore
        )
        # res = Pos(X=-mv) * sk
        res = Pos(X=hor_offst) * sk
        return Pos(Y=vertical_offst) * res

    def sk_h(self, scalar: float = 1, r: float = 1, w_scale: float = 1) -> Sketch:

        cal_blade_thick = self.blade_thick * scalar
        res = RectangleRounded(
            self.width * w_scale,
            cal_blade_thick,
            r,
            align=(Align.CENTER, Align.MIN, Align.CENTER),
        )  # type:ignore
        return res

    def sk_init(self, r: float = 4, scalar: float = 2) -> Sketch:
        hor_sketch = self.sk_h(scalar=scalar, r=r)
        return Rot(X=20) * hor_sketch

    def tip(self) -> Part:
        tip_body = Cylinder(
            radius=0.15 * self.blade_thick,
            height=0.69 * self.width,
            rotation=(90, 0, 0),
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
        tip_filleted = chamfer(tip_body.edges(), 0.5)
        return (self.wire ^ 1) * tip_filleted  # type:ignore

    def blade_h(self) -> Part:
        """
        Loft horizontal blade && tip
        """
        show_object(self.wire, name="wire")
        rrt = Rot(Z=-90)
        body_sections = [
            (self.wire ^ 0) * rrt * self.sk_init(),
            (self.wire ^ 0.25) * Pos(X=4) * rrt * self.sk_h(0.5, w_scale=0.92),
            (self.wire ^ 0.69) * Pos(X=2) * rrt * self.sk_h(0.3),
            (self.wire ^ 0.8) * Pos(X=0) * rrt * self.sk_h(0.3),
            (self.wire ^ 1) * Pos(X=-1) * rrt * self.sk_h(0.2, w_scale=0.69),
        ]
        # res = loft(body_sections)
        res = sweep(sections=body_sections, path=self.wire, multisection=True)
        return res

    def blade_v(self) -> Part:
        rrt = Rot(Z=-90)
        hor_offst = 6
        body_sections = [
            (self.wire ^ 0) * rrt * self.sk_init(),
            (self.wire ^ 0.25)
            * rrt
            * self.sk_v(scalar=1, vertical_offst=-4, hor_offst=hor_offst, r=1),
            (self.wire ^ 0.69)
            * rrt
            * self.sk_v(scalar=0.7, vertical_offst=-2, hor_offst=hor_offst, r=1),
            (self.wire ^ 0.85)
            * rrt
            * self.sk_v(scalar=0.2, vertical_offst=-1, hor_offst=hor_offst, r=0.5),
        ]
        res = loft(body_sections)
        # res = sweep(sections=body_sections, path=self.wire, multisection=True)

        return res

    def cam(self, angle: float = 2.5):
        """
        I have to use complicated way to place cam body on the wire
        """
        cam_r = 5.65 / 2
        cam_lip = cam_r - 0.75
        cam_wire_inset = self.cam_wire_r / 3

        x_offset = self.width / 2 - self.blade_lip_width / 2
        wha = Pos(Z=-x_offset, X=-self.cam_l * 0.33) * self.wire ^ 0.62
        # wha = self.wire ^ 0.61

        # sk = Pos(Y=-x_offset, Z=self.cam_l * 0.69) * Circle(cam_r)
        sk = Circle(cam_r)
        cam_body = wha * extrude(sk, amount=self.cam_l, taper=0.75)
        cam_face = cam_body.faces()[0]
        cam_body += Plane(cam_face) * extrude(Circle(cam_lip), amount=10, taper=2)
        ## Incision for cam wire
        incision = Plane(cam_face) * extrude(
            Rectangle(10, self.cam_wire_r * 2, align=(Align.MAX, Align.CENTER)),
            amount=-self.cam_l,
            both=True,
        )
        cam_body_rotated = extrude(
            Pos(Z=-0.1) * Rot(X=-angle) * sk, amount=self.cam_l, taper=0.75
        )
        incision_rotated = extrude(
            Rot(X=-angle)
            * Rectangle(10, self.cam_wire_r * 2, align=(Align.MAX, Align.CENTER)),
            amount=self.cam_l,
        )
        cam_body_rotated += incision_rotated
        cam_wire_body = [
            (self.wire ^ 0) * Pos(X=cam_wire_inset) * Circle(self.cam_wire_r),
            (self.wire ^ 0.15)
            * Pos(Y=-x_offset, X=-4 + cam_wire_inset)
            * Circle(self.cam_wire_r),
            (self.wire ^ 0.4)
            * Pos(Y=-x_offset, X=-2 + cam_wire_inset)
            * Circle(self.cam_wire_r),
            wha * Pos(X=0, Z=0) * Circle(4 / 2),  ## -self.cam_l/4)
        ]

        cam_wire_body = loft(cam_wire_body)
        cam_wire_hand = sweep(
            (self.hand_wire ^ 0) * Pos(X=cam_wire_inset) * Circle(self.cam_wire_r),
            path=self.hand_wire,
        )
        # show_object(cam_wire_hand, name="cam_wire_hand")

        addendum = extrude(
            (self.wire ^ 0) * Pos(X=cam_wire_inset) * Circle(self.cam_wire_r),
            amount=-30,
            taper=-0.5,
        )
        # show_object(addendum, name="addendum")
        res = wha * cam_body_rotated
        res += cam_wire_body + incision + addendum + cam_wire_hand

        return res

    def sk_hand(self, finged: bool = False) -> Sketch:

        res = RectangleRounded(
            self.hand_w, self.width, 4, align=(Align.MIN, Align.CENTER, Align.CENTER)
        )  # type:ignore
        if finged:
            fing_incision = 5
            res = Pos(X=-fing_incision) * RectangleRounded(
                self.hand_w - fing_incision,
                self.width,
                4,
                align=(Align.MIN, Align.CENTER, Align.CENTER),
            )  # type:ignore
        return res

    @property
    def hand_wire(self) -> Spline:
        pnts = [
            (5, 0),
            # (-10, 30),
            (10, 60),
            (35, 105),
        ]
        # show_object(self.hand_wire, name="hand_wire")
        return Spline(pnts)

    def hand(self):
        body_sections = [
            (self.wire ^ 0) * Rot(Z=-90) * self.sk_init(),
            (self.hand_wire ^ 0.3) * self.sk_hand(finged=False),
            (self.hand_wire ^ 0.75) * self.sk_hand(finged=False),
            # (hand_wire ^ 0.65)  *  self.sk_hand(finged = False),
            # (hand_wire ^ 0.95) * self.sk_hand(finged=False),
            (self.hand_wire ^ 1) * self.sk_hand(finged=False),
        ]
        res = loft(body_sections)
        # res = sweep(sections=body_sections, path=hand_wire, multisection=True)
        return res

    def build(self):
        """
        Build the blade by defining different 2D "Sketch" positions on given curve aka his majesty-"self.wire"
        """
        # dev zone
        blade = self.blade_h() + self.blade_v() + self.hand() + self.tip()
        cam = self.cam()
        blade -= cam
        show_object(cam, name="cam")
        return blade


blade_spline = Spline(BLADE_N4_PTS)
test = Base_Blade(wire=blade_spline)
show_object(test.hand(), name="hand")
show_object(test.build(), name="build")

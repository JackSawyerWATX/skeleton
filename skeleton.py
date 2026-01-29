import math
import pygame
from typing import List, Tuple

WIDTH = 800
HEIGHT = 600


class IKSolver:
  """Handles inverse kinematics calculations (Single Responsibility)"""
  
  @staticmethod
  def solve_two_bone(hip: pygame.Vector2, target: pygame.Vector2,
                     upper_len: float, lower_len: float,
                     bend_dir: pygame.Vector2) -> Tuple[pygame.Vector2, pygame.Vector2]:
    to_target = target - hip
    dist = to_target.length()
    if dist == 0:
      return hip, hip
    max_reach = upper_len + lower_len - 0.001
    dist = min(dist, max_reach)
    direction = to_target.normalize()

    a = upper_len
    b = lower_len
    c = dist
    cos_angle = max(-1.0, min(1.0, (a * a + c * c - b * b) / (2 * a * c)))
    angle = math.acos(cos_angle)

    perp = pygame.Vector2(-direction.y, direction.x)
    if perp.dot(bend_dir) < 0:
      perp = -perp

    knee = hip + direction * (math.cos(angle) * a) + perp * (math.sin(angle) * a)
    return knee, target


class Leg:
  """Represents a lizard leg with stepping behavior (Single Responsibility)"""
  
  def __init__(self, side: int, anchor_idx: int, is_front: bool) -> None:
    self.side = side
    self.anchor_idx = anchor_idx
    self.is_front = is_front
    self.foot = pygame.Vector2(0, 0)
    self.rest_offset = pygame.Vector2(60 * self.side, -20 if self.is_front else 30)
    self.upper_len = 28 if self.is_front else 30
    self.lower_len = 26 if self.is_front else 28
    self.phase = 0.0
    self.planted = True
    self.step_progress = 1.0
    self.step_from = pygame.Vector2(0, 0)
    self.step_to = pygame.Vector2(0, 0)
  
  def get_max_reach(self) -> float:
    return self.upper_len + self.lower_len - 5


class Spine:
  """Manages the spine/backbone of the lizard (Single Responsibility)"""
  
  def __init__(self, num_vertebrae: int, segment_length: float, start_pos: pygame.Vector2):
    self.num_vertebrae = num_vertebrae
    self.segment_length = segment_length
    self.vertebrae = [start_pos.copy() for _ in range(num_vertebrae)]
  
  def update(self, head_pos: pygame.Vector2) -> None:
    """Update spine to follow head position"""
    self.vertebrae[0] = head_pos
    for i in range(1, self.num_vertebrae):
      direction = self.vertebrae[i] - self.vertebrae[i - 1]
      if direction.length_squared() == 0:
        direction = pygame.Vector2(1, 0)
      self.vertebrae[i] = self.vertebrae[i - 1] + direction.normalize() * self.segment_length
  
  def get_body_direction(self) -> pygame.Vector2:
    """Get the forward direction of the body"""
    body_dir = self.vertebrae[0] - self.vertebrae[3]
    if body_dir.length_squared() == 0:
      body_dir = pygame.Vector2(1, 0)
    return body_dir.normalize()
  
  def get_side_direction(self) -> pygame.Vector2:
    """Get the perpendicular (side) direction"""
    body_dir = self.get_body_direction()
    return pygame.Vector2(-body_dir.y, body_dir.x)


class LegController:
  """Controls leg movement and stepping logic (Single Responsibility)"""
  
  def __init__(self, legs: List[Leg]):
    self.legs = legs
  
  def update(self, spine: Spine, move_speed: float, move_dir: pygame.Vector2) -> None:
    """Update all leg positions and handle stepping"""
    body_dir = spine.get_body_direction()
    side_dir = spine.get_side_direction()
    
    for leg in self.legs:
      anc = spine.vertebrae[leg.anchor_idx]
      max_reach = leg.get_max_reach()
      
      ideal_foot = self._calculate_ideal_foot_position(leg, anc, side_dir, body_dir, max_reach)
      
      if leg.planted:
        self._handle_planted_leg(leg, anc, side_dir, body_dir, move_speed, move_dir, 
                                 ideal_foot, max_reach)
      else:
        self._handle_stepping_leg(leg)
  
  def _calculate_ideal_foot_position(self, leg: Leg, anchor: pygame.Vector2,
                                     side_dir: pygame.Vector2, body_dir: pygame.Vector2,
                                     max_reach: float) -> pygame.Vector2:
    """Calculate where the foot should ideally be"""
    rest_pos = anchor + side_dir * (leg.side * 38)
    forward = body_dir * (15 if leg.is_front else -10)
    ideal_foot = rest_pos + forward
    
    to_ideal = ideal_foot - anchor
    if to_ideal.length() > max_reach:
      ideal_foot = anchor + to_ideal.normalize() * max_reach
    
    return ideal_foot
  
  def _handle_planted_leg(self, leg: Leg, anchor: pygame.Vector2, side_dir: pygame.Vector2,
                         body_dir: pygame.Vector2, move_speed: float, move_dir: pygame.Vector2,
                         ideal_foot: pygame.Vector2, max_reach: float) -> None:
    """Handle logic for a planted (stationary) leg"""
    to_foot = leg.foot - anchor
    lateral = to_foot.dot(side_dir)
    longitudinal = to_foot.dot(body_dir)
    
    # Emergency step if foot crosses to wrong side
    if lateral * leg.side < 0:
      self._trigger_emergency_step(leg, anchor, side_dir, move_dir, body_dir, max_reach)
      return
    
    # Enforce minimum lateral distance
    min_lateral = 25
    if lateral * leg.side < min_lateral:
      lateral = min_lateral * leg.side
      leg.foot = anchor + side_dir * lateral + body_dir * longitudinal
      to_foot = leg.foot - anchor
    
    # Clamp to max reach
    dist = to_foot.length()
    if dist > max_reach:
      leg.foot = anchor + to_foot.normalize() * max_reach
    
    # Check if step is needed
    dist_from_ideal = leg.foot.distance_to(ideal_foot)
    other_leg_moving = self._check_diagonal_leg_moving(leg)
    
    if move_speed > 0.5 and dist_from_ideal > 20 and not other_leg_moving:
      self._trigger_step(leg, anchor, side_dir, body_dir, move_dir, ideal_foot, max_reach)
  
  def _trigger_emergency_step(self, leg: Leg, anchor: pygame.Vector2, side_dir: pygame.Vector2,
                              move_dir: pygame.Vector2, body_dir: pygame.Vector2,
                              max_reach: float) -> None:
    """Trigger an emergency step when foot crosses centerline"""
    leg.planted = False
    leg.step_progress = 0.0
    leg.step_from = leg.foot.copy()
    step_forward = move_dir * 60 if move_dir.length() > 0 else body_dir * 30
    leg.step_to = anchor + side_dir * (leg.side * 30) + step_forward
    
    to_final = leg.step_to - anchor
    if to_final.length() > max_reach:
      leg.step_to = anchor + to_final.normalize() * max_reach
  
  def _trigger_step(self, leg: Leg, anchor: pygame.Vector2, side_dir: pygame.Vector2,
                   body_dir: pygame.Vector2, move_dir: pygame.Vector2,
                   ideal_foot: pygame.Vector2, max_reach: float) -> None:
    """Trigger a normal step"""
    leg.planted = False
    leg.step_progress = 0.0
    leg.step_from = leg.foot.copy()
    
    step_forward = move_dir * 45
    step_target = ideal_foot + step_forward
    
    to_target = step_target - anchor
    target_lateral = to_target.dot(side_dir)
    target_long = to_target.dot(body_dir)
    
    if target_lateral * leg.side < 28:
      target_lateral = 28 * leg.side
    
    leg.step_to = anchor + side_dir * target_lateral + body_dir * target_long
    
    to_final = leg.step_to - anchor
    if to_final.length() > max_reach:
      leg.step_to = anchor + to_final.normalize() * max_reach
  
  def _handle_stepping_leg(self, leg: Leg) -> None:
    """Handle animation of a leg in mid-step"""
    leg.step_progress = min(1.0, leg.step_progress + 0.15)
    t = leg.step_progress
    lift = math.sin(t * math.pi) * 15
    leg.foot = leg.step_from.lerp(leg.step_to, t) + pygame.Vector2(0, -lift)
    
    if leg.step_progress >= 1.0:
      leg.planted = True
      leg.foot = leg.step_to.copy()
  
  def _check_diagonal_leg_moving(self, leg: Leg) -> bool:
    """Check if diagonal counterpart leg is moving"""
    return any(
      not other.planted for other in self.legs
      if (other.is_front == leg.is_front and other.side != leg.side) or
         (other.is_front != leg.is_front and other.side == leg.side)
    )


class Renderer:
  """Handles all drawing operations (Single Responsibility)"""
  
  def __init__(self, screen: pygame.Surface):
    self.screen = screen
    self.bg_color = (30, 30, 30)
  
  def clear(self) -> None:
    self.screen.fill(self.bg_color)
  
  def draw_ribs(self, spine: Spine) -> None:
    """Draw the ribcage"""
    for i in range(2, spine.num_vertebrae - 3):
      rib_center = spine.vertebrae[i]
      if i < len(spine.vertebrae) - 1:
        rib_dir = spine.vertebrae[i + 1] - spine.vertebrae[i - 1]
        if rib_dir.length_squared() == 0:
          rib_dir = pygame.Vector2(1, 0)
        rib_dir = rib_dir.normalize()
        rib_side = pygame.Vector2(-rib_dir.y, rib_dir.x)
        
        if i <= 16:
          rib_length = 26 - abs(i - 11) * 0.8
        else:
          rib_length = max(3, 18 - (i - 16) * 0.9)
        
        rib_left = rib_center + rib_side * rib_length
        rib_right = rib_center - rib_side * rib_length
        pygame.draw.line(self.screen, (180, 180, 180), rib_center, rib_left, 2)
        pygame.draw.line(self.screen, (180, 180, 180), rib_center, rib_right, 2)
  
  def draw_spine(self, spine: Spine) -> None:
    """Draw the spine vertebrae"""
    for i in range(1, spine.num_vertebrae):
      pygame.draw.line(self.screen, (200, 200, 200), spine.vertebrae[i - 1], 
                      spine.vertebrae[i], 4)
    for i in range(1, spine.num_vertebrae):
      pygame.draw.circle(self.screen, (240, 220, 120), spine.vertebrae[i], 3)
  
  def draw_skull(self, spine: Spine) -> None:
    """Draw the skull"""
    head_dir = spine.vertebrae[0] - spine.vertebrae[1]
    if head_dir.length_squared() == 0:
      head_dir = pygame.Vector2(1, 0)
    head_dir = head_dir.normalize()
    
    head_center = spine.vertebrae[0] + head_dir * 14
    head_side_dir = pygame.Vector2(-head_dir.y, head_dir.x)
    nose = head_center + head_dir * 32
    back_left = head_center - head_dir * 14 + head_side_dir * 22
    back_right = head_center - head_dir * 14 - head_side_dir * 22
    
    pygame.draw.polygon(self.screen, (245, 235, 200), [nose, back_left, back_right])
    pygame.draw.polygon(self.screen, (220, 210, 180), [nose, back_left, back_right], 2)
    
    # Eyes
    eye_offset = head_dir * 10
    eye_side = head_side_dir * 9
    left_eye = head_center + eye_offset + eye_side
    right_eye = head_center + eye_offset - eye_side
    pygame.draw.circle(self.screen, (30, 30, 30), left_eye, 4)
    pygame.draw.circle(self.screen, (30, 30, 30), right_eye, 4)
  
  def draw_leg(self, leg: Leg, spine: Spine, ik_solver: IKSolver) -> None:
    """Draw a single leg with bones and claws"""
    anc = spine.vertebrae[leg.anchor_idx]
    body_dir = spine.get_body_direction()
    side_dir = spine.get_side_direction()
    
    bend_dir = side_dir * leg.side + (-body_dir if leg.is_front else body_dir) * 0.5
    if bend_dir.length_squared() == 0:
      bend_dir = side_dir * leg.side
    
    knee, foot = ik_solver.solve_two_bone(anc, leg.foot, leg.upper_len, leg.lower_len, bend_dir)
    
    self._draw_bone_segment(anc, knee, leg.upper_len, 3, 2)
    self._draw_bone_segment(knee, foot, leg.lower_len, 3, 2)
    
    pygame.draw.circle(self.screen, (220, 220, 220), anc, 4)
    pygame.draw.circle(self.screen, (220, 220, 220), knee, 4)
    pygame.draw.circle(self.screen, (220, 220, 220), foot, 4)
    
    self._draw_claws(foot, knee)
  
  def _draw_bone_segment(self, start: pygame.Vector2, end: pygame.Vector2,
                         length: float, width_start: float, width_mid: float) -> None:
    """Draw a bone-shaped segment"""
    bone_dir = end - start
    if bone_dir.length() > 0:
      bone_dir = bone_dir.normalize()
      perp = pygame.Vector2(-bone_dir.y, bone_dir.x)
      mid = start + bone_dir * (length / 2)
      
      pygame.draw.polygon(self.screen, (200, 200, 200), [
        start + perp * width_start, start - perp * width_start,
        mid - perp * width_mid, mid + perp * width_mid
      ])
      pygame.draw.polygon(self.screen, (200, 200, 200), [
        mid + perp * width_mid, mid - perp * width_mid,
        end - perp * width_start, end + perp * width_start
      ])
  
  def _draw_claws(self, foot: pygame.Vector2, knee: pygame.Vector2) -> None:
    """Draw claws at the foot"""
    foot_dir = foot - knee
    if foot_dir.length() > 0:
      foot_dir = foot_dir.normalize()
      
      for angle_offset in [-0.4, -0.15, 0.15, 0.4]:
        angle = math.atan2(foot_dir.y, foot_dir.x) + angle_offset
        claw_dir = pygame.Vector2(math.cos(angle), math.sin(angle))
        claw_tip = foot + claw_dir * 8
        claw_base_left = foot + claw_dir * 2 + pygame.Vector2(-claw_dir.y, claw_dir.x) * 1.5
        claw_base_right = foot + claw_dir * 2 - pygame.Vector2(-claw_dir.y, claw_dir.x) * 1.5
        pygame.draw.polygon(self.screen, (240, 240, 240), 
                          [claw_tip, claw_base_left, claw_base_right])


class LizardSkeleton:
  """Main lizard skeleton entity (coordinates all components)"""
  
  def __init__(self, start_pos: pygame.Vector2):
    self.spine = Spine(35, 18, start_pos)
    self.legs = self._create_legs()
    self.leg_controller = LegController(self.legs)
    self.ik_solver = IKSolver()
    self._initialize_leg_positions()
  
  def _create_legs(self) -> List[Leg]:
    """Create the four legs"""
    legs = [
      Leg(-1, 4, True),   # Front left
      Leg(1, 4, True),    # Front right
      Leg(-1, 14, False), # Back left
      Leg(1, 14, False)   # Back right
    ]
    legs[0].phase = 0.0
    legs[1].phase = 0.5
    legs[2].phase = 0.5
    legs[3].phase = 0.0
    return legs
  
  def _initialize_leg_positions(self) -> None:
    """Set initial foot positions"""
    for leg in self.legs:
      anc = self.spine.vertebrae[leg.anchor_idx]
      leg.foot = anc + leg.rest_offset
      leg.step_from = leg.foot
      leg.step_to = leg.foot
  
  def update(self, head_pos: pygame.Vector2, move_speed: float, move_dir: pygame.Vector2) -> None:
    """Update the entire skeleton"""
    self.spine.update(head_pos)
    self.leg_controller.update(self.spine, move_speed, move_dir)
  
  def draw(self, renderer: Renderer) -> None:
    """Draw the entire skeleton"""
    renderer.draw_ribs(self.spine)
    renderer.draw_spine(self.spine)
    renderer.draw_skull(self.spine)
    
    for leg in self.legs:
      renderer.draw_leg(leg, self.spine, self.ik_solver)


def main():
  pygame.init()
  screen = pygame.display.set_mode((WIDTH, HEIGHT))
  pygame.display.set_caption("Skeleton")
  clock = pygame.time.Clock()
  
  # Initialize components
  start_pos = pygame.Vector2(WIDTH // 2, HEIGHT // 2)
  lizard = LizardSkeleton(start_pos)
  renderer = Renderer(screen)
  
  # Input tracking
  prev_mouse = pygame.Vector2(pygame.mouse.get_pos())
  running = True
  
  while running:
    # Handle events
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False
    
    # Calculate mouse movement
    mouse_pos = pygame.Vector2(pygame.mouse.get_pos())
    mouse_delta = mouse_pos - prev_mouse
    prev_mouse = mouse_pos
    
    move_speed = mouse_delta.length()
    move_dir = pygame.Vector2(0, 0)
    if move_speed > 0.1:
      move_dir = mouse_delta.normalize()
    
    # Update and render
    lizard.update(mouse_pos, move_speed, move_dir)
    
    renderer.clear()
    lizard.draw(renderer)
    
    pygame.display.flip()
    clock.tick(60)
  
  pygame.quit()


if __name__ == "__main__":
  main()

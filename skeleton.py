import math
import pygame

WIDTH = 800
HEIGHT = 600


class Leg:
  def __init__(self, side: int, anchor_idx: int, is_front: bool) -> None:
    self.side = side
    self.anchor_idx = anchor_idx
    self.is_front = is_front
    self.foot = pygame.Vector2(0, 0)
    self.rest_offset = pygame.Vector2(
      60 * self.side,
      -20 if self.is_front else 30
    )
    self.upper_len = 28 if self.is_front else 30
    self.lower_len = 26 if self.is_front else 28
    self.phase = 0.0
    self.planted = True
    self.step_progress = 1.0
    self.step_from = pygame.Vector2(0, 0)
    self.step_to = pygame.Vector2(0, 0)


def solve_two_bone(hip: pygame.Vector2, target: pygame.Vector2,
                   upper_len: float, lower_len: float,
                   bend_dir: pygame.Vector2) -> tuple[pygame.Vector2, pygame.Vector2]:
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


def main():
  pygame.init()
  screen = pygame.display.set_mode((WIDTH, HEIGHT))
  pygame.display.set_caption("Skeleton")
  clock = pygame.time.Clock()

  num_vertebrate = 22
  segment_length = 18
  spine = [pygame.Vector2(WIDTH//2, HEIGHT//2)
           for _ in range(num_vertebrate)]
  angle = [0.0] * num_vertebrate

  legs = []
  legs.append(Leg(-1, 4, True))
  legs.append(Leg(1, 4, True))
  legs.append(Leg(-1, 14, False))
  legs.append(Leg(1, 14, False))

  legs[0].phase = 0.0
  legs[1].phase = 0.5
  legs[2].phase = 0.5
  legs[3].phase = 0.0

  for l in legs:
    anc = spine[l.anchor_idx]
    l.foot = anc + l.rest_offset
    l.step_from = l.foot
    l.step_to = l.foot

  prev_mouse = pygame.Vector2(pygame.mouse.get_pos())
  running = True
  while running:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        running = False

    mouse_pos = pygame.Vector2(pygame.mouse.get_pos())
    mouse_delta = mouse_pos - prev_mouse
    prev_mouse = mouse_pos

    spine[0] = mouse_pos
    for i in range(1, num_vertebrate):
      direction = spine[i] - spine[i - 1]
      if direction.length_squared() == 0:
        direction = pygame.Vector2(1, 0)
      spine[i] = spine[i - 1] + direction.normalize() * segment_length

    body_dir = spine[0] - spine[3]
    if body_dir.length_squared() == 0:
      body_dir = pygame.Vector2(1, 0)
    body_dir = body_dir.normalize()
    side_dir = pygame.Vector2(-body_dir.y, body_dir.x)

    move_speed = mouse_delta.length()
    move_dir = pygame.Vector2(0, 0)
    if move_speed > 0.1:
      move_dir = mouse_delta.normalize()

    for l in legs:
      anc = spine[l.anchor_idx]
      max_reach = l.upper_len + l.lower_len - 5
      
      rest_pos = anc + side_dir * (l.side * 35)
      forward = body_dir * (15 if l.is_front else -10)
      ideal_foot = rest_pos + forward
      
      to_ideal = ideal_foot - anc
      if to_ideal.length() > max_reach:
        ideal_foot = anc + to_ideal.normalize() * max_reach

      if l.planted:
        to_foot = l.foot - anc
        dist_from_anchor = to_foot.length()
        
        if dist_from_anchor > max_reach:
          l.foot = anc + to_foot.normalize() * max_reach
          to_foot = l.foot - anc
        
        lateral_proj = to_foot.dot(side_dir)
        if lateral_proj * l.side < 0:
          to_foot = to_foot - side_dir * (2 * lateral_proj)
          l.foot = anc + to_foot
          if to_foot.length() > max_reach:
            l.foot = anc + to_foot.normalize() * max_reach
        
        dist_from_ideal = l.foot.distance_to(ideal_foot)
        
        other_leg_moving = any(
          not other.planted for other in legs
          if (other.is_front == l.is_front and other.side != l.side) or
             (other.is_front != l.is_front and other.side == l.side)
        )
        
        if move_speed > 0.5 and dist_from_ideal > 20 and not other_leg_moving:
          l.planted = False
          l.step_progress = 0.0
          l.step_from = l.foot.copy()
          step_forward = move_dir * 70
          step_target = ideal_foot + step_forward
          
          to_target = step_target - anc
          target_lateral = to_target.dot(side_dir)
          if target_lateral * l.side < 0:
            to_target = to_target - side_dir * (2 * target_lateral)
          
          l.step_to = anc + to_target
          if to_target.length() > max_reach:
            l.step_to = anc + to_target.normalize() * max_reach
      else:
        l.step_progress = min(1.0, l.step_progress + 0.15)
        t = l.step_progress
        lift = math.sin(t * math.pi) * 15
        l.foot = l.step_from.lerp(l.step_to, t) + pygame.Vector2(0, -lift)
        if l.step_progress >= 1.0:
          l.planted = True
          l.foot = l.step_to.copy()

    screen.fill((30, 30, 30))

    for i in range(1, num_vertebrate):
      pygame.draw.line(screen, (200, 200, 200), spine[i - 1], spine[i], 4)
    for i in range(1, num_vertebrate):
      pygame.draw.circle(screen, (240, 220, 120), spine[i], 3)

    head_dir = spine[0] - spine[1]
    if head_dir.length_squared() == 0:
      head_dir = pygame.Vector2(1, 0)
    head_dir = head_dir.normalize()
    head_center = spine[0] + head_dir * 10
    head_side_dir = pygame.Vector2(-head_dir.y, head_dir.x)
    nose = head_center + head_dir * 22
    back_left = head_center - head_dir * 10 + head_side_dir * 16
    back_right = head_center - head_dir * 10 - head_side_dir * 16
    pygame.draw.polygon(screen, (245, 235, 200), [nose, back_left, back_right])
    pygame.draw.polygon(screen, (220, 210, 180), [nose, back_left, back_right], 2)

    eye_offset = head_dir * 6
    eye_side = head_side_dir * 6
    left_eye = head_center + eye_offset + eye_side
    right_eye = head_center + eye_offset - eye_side
    pygame.draw.circle(screen, (30, 30, 30), left_eye, 3)
    pygame.draw.circle(screen, (30, 30, 30), right_eye, 3)

    for l in legs:
      anc = spine[l.anchor_idx]
      bend_dir = side_dir * l.side + (-body_dir if l.is_front else body_dir) * 0.5
      if bend_dir.length_squared() == 0:
        bend_dir = side_dir * l.side
      knee, foot = solve_two_bone(anc, l.foot, l.upper_len, l.lower_len, bend_dir)
      pygame.draw.line(screen, (120, 200, 255), anc, knee, 3)
      pygame.draw.line(screen, (120, 200, 255), knee, foot, 3)
      pygame.draw.circle(screen, (120, 200, 255), knee, 4)
      pygame.draw.circle(screen, (120, 200, 255), foot, 4)

    pygame.display.flip()
    clock.tick(60)

  pygame.quit()


if __name__ == "__main__":
  main()

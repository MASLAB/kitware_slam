def sign(num):
  return num / abs(num)


class PracticalPID():
  def __init__(self, p_gain, i_gain, d_gain, max_gain, mid_gain, min_gain, update_time):
    self.p_gain = p_gain
    self.i_gain = i_gain
    self.d_gain = d_gain
    self.max_gain = abs(max_gain)
    self.mid_gain = abs(mid_gain)
    self.min_gain = abs(min_gain)
    self.update_time = update_time

    self.old_p_term = 0.0
    self.i_term = 0.0

  def reset(self):
    self.old_p_term = 0.0
    self.i_term = 0.0


  def cap(self, term):
    if abs(term) > self.max_gain:
      if term > 0:
        return self.max_gain
      else:
        return -self.max_gain
    else:
      return term


  def update(self, error, tolerance):
    done = True
    gain = 0.0

    if abs(error) > abs(tolerance):
      done = False
      p_term       = self.cap(self.p_gain * error)
      d_term       = self.d_gain * (p_term - self.old_p_term) / self.update_time
      if abs(p_term) < self.mid_gain:
        self.i_term += self.i_gain * p_term * self.update_time
        self.i_term = self.cap(self.i_term)
      self.old_p_term = p_term
      gain = p_term + d_term + self.i_term
      gain += sign(gain) * self.min_gain
      gain = self.cap(gain)
    else:
      self.reset()

    return (gain, done)
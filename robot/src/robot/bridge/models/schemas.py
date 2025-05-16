from pydantic import BaseModel


class TokenResponse(BaseModel):
    access_token: str
    token_type: str = "Bearer"


class UserInfo(BaseModel):
    username: str
    email: str | None = None
    full_name: str | None = None


class HeadRotation(BaseModel):
    rotation_percent: float = 0.0

    def __eq__(self, other):
        if not isinstance(other, HeadRotation):
            return False
        return self.rotation_percent == other.rotation_percent


class ChassisMovement(BaseModel):
    left_wheels_speed: float = 0.0
    right_wheels_speed: float = 0.0

    def __eq__(self, other):
        if not isinstance(other, ChassisMovement):
            return False
        return (self.left_wheels_speed == other.left_wheels_speed and 
                self.right_wheels_speed == other.right_wheels_speed)

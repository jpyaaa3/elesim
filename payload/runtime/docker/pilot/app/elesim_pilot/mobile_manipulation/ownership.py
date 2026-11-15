"""One parent workflow, one authorized arm/base child. No force acquisition."""
from dataclasses import dataclass, field
import uuid


@dataclass(frozen=True)
class Grant:
    generation: int
    owner: str
    route: tuple
    channels: frozenset
    token: str = field(default_factory=lambda: uuid.uuid4().hex)


class Ownership:
    def __init__(self):self.generation=0;self.session=None;self.child=None
    def acquire(self,route):
        if self.session is not None:raise RuntimeError('mobile task already owns motion')
        self.generation+=1;self.session=uuid.uuid4().hex;self.route=tuple(route)
    def grant(self,owner,channels):
        if self.session is None or self.child is not None:raise RuntimeError('motion grant unavailable')
        self.child=Grant(self.generation,owner,self.route,frozenset(channels));return self.child
    def require(self,grant,route,channel):
        if grant!=self.child or grant.generation!=self.generation or tuple(route)!=self.route or channel not in grant.channels:
            raise RuntimeError('stale or conflicting workflow motion grant')
    def finish(self,grant):
        if self.child!=grant:raise RuntimeError('wrong child motion grant')
        self.child=None
    def release(self):
        self.generation+=1;self.child=None;self.session=None

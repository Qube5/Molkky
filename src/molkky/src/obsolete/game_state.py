from constants import winning_score
from constants import back_score

class GameState:
    def __init__(self, num_players):
        self.num_players = num_players
        self.scores = [0] * num_players
        self.turn = 0

    def update_score(self, new_score):
        self.scores[self.turn] = new_score

    def increment_score(self, points_gained):
        curr_score = self.scores[self.turn]
        score = curr_score + points_gained
        score = score if score <= winning_score else back_score
        self.scores[self.turn] = score

    def next_turn(self):
        self.turn = (self.turn + 1) % self.num_players

    def is_game_over(self):
        winner = -1
        for i in range(self.num_players):
            if self.scores[i] == winning_score:
                winner = i
        return winner

    def get_current_turn(self):
        return self.turn

    def get_num_players(self):
        return self.num_players

    def get_scores(self):
        return self.scores

    def get_player_score(self, player_index):
        return self.scores[player_index]

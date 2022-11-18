class AppException(Exception):
    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args)
        self.error = kwargs['error']
        self.message = kwargs['message']

    def __str__(self) -> str:
        return f'[ERROR] {self.error}: {self.message}'
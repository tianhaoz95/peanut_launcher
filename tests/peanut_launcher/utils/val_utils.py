def is_valid_output_checker(output_checker):
    fake_output = 'This is a fake output'
    try:
        checker_result = output_checker(fake_output)
        if type(checker_result) == bool:
            return True
        else:
            return False
    except:
        return False
